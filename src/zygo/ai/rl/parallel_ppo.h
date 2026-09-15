#pragma once

// PPO over several environments at once, one thread per environment.
//
// Same algorithm and the same PpoConfig as PpoTrainer; what changes is who does the work.
//
// COLLECTING
//   Every worker owns an environment, a private copy of the network and its own rollout buffer, and steps them on its own thread.
//   Nothing is shared while a rollout is collected: the weights and the observation normalizer are copied into the workers once,
//   at the start of each advance(), and stay frozen until the rollout is full. No locks, no per-step synchronization.
//
// UPDATING
//   Physics is not the only cost. With the reduced-coordinate solver one step of the dog is ~0.07 ms, and the PPO update
//   (5 epochs over 4096 samples, sample-by-sample forward/backward) takes about twice as long as collecting the rollout.
//   Parallelizing only the robots would stop at ~1.5x. So every minibatch is split across the workers too:
//   each computes the gradient of its slice on its own copy of the net, the main thread sums them, clips, takes ONE Adam step
//   and the workers pick up the new weights at the start of the next slice.
//
// WHAT IS DIFFERENT FROM PpoTrainer, ON PURPOSE
//   - config.rollout_steps is the TOTAL per update, split evenly between the environments (rounded up to a multiple of their count).
//   - The observation normalizer is frozen during a rollout and updated from all of it right AFTER the update. The update therefore
//     sees the same normalized inputs the actions were sampled with, and the probability ratio starts at exactly 1.
//   - GAE runs separately on each environment's stream, each with its own bootstrap. Advantages are normalized over ALL of them.
//   - A time-limit cut or an outside reset bootstraps with V(the state the episode was cut in) - see RolloutBuffer::bootstrapLastStep().
//
// RANDOM NUMBERS
//   Random is thread_local. Every worker thread seeds its own stream from (seed, worker index), so environment k always sees the same
//   terrain, friction and exploration noise, whatever else the main thread does with Random. The minibatch shuffle uses a generator
//   owned by the trainer. A run is reproducible for a fixed seed and a fixed number of environments.
//
// THREADS
//   envs.size() worker threads. The calling thread only coordinates and waits, so it stays free for drawing between advance() calls.
//   Everything public must be called from ONE thread (the one that owns the trainer) - never from inside an environment.

#include "ppo.h"
#include "environment.h"
#include "rollout_buffer.h"
#include <zygo/ai/nn/actor_critic.h>
#include <zygo/ai/nn/optimizer.h>

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>


namespace zygo {
namespace rl {

class ParallelPpoTrainer
{
private:
  struct Worker
  {
    int                                  index = 0;
    IEnvironment                       * env   = nullptr;
    std::unique_ptr<nn::ActorCriticNet>  net;
    std::vector<nn::ParamBlock>          params;
    RolloutBuffer                        buffer;

    std::vector<Real> obs, next_obs, env_action, action, eps, probs, d_policy, d_log_std;

    Real cur_episode_return = REAL_ZERO;
    int  cur_episode_length = 0;
    Real bootstrap_value    = REAL_ZERO;

    std::vector<Real> finished_returns; // since the last takeFinishedEpisodes()

    // episodes finished during the current rollout, for PpoStats
    double rollout_return_sum = 0.0;
    long   rollout_length_sum = 0;
    int    rollout_episodes   = 0;

    // accumulated over one minibatch slice
    double policy_loss = 0.0, value_loss = 0.0, entropy = 0.0, kl = 0.0, clipped = 0.0;
    int    samples = 0;

    Worker( int capacity, int obs_size, int stored_action_size ) : buffer( capacity, obs_size, stored_action_size ) {}
  };

  PpoConfig  config;
  PolicyKind policy_kind;

  nn::ActorCriticNet&         net;        // the master: the optimizer steps these weights
  nn::AdamOptimizer           optimizer;
  std::vector<nn::ParamBlock> param_blocks;

  std::vector<std::unique_ptr<Worker>> workers;

  int  steps_per_env;
  int  max_steps_in_episode;
  Real adv_mean;
  Real adv_inv_std;

  std::vector<int> order;   // global sample index = worker * steps_per_env + step
  u64              shuffle_state;

  long long total_env_steps;
  int       updates_done;
  PpoStats  last_stats;

  // ---- thread pool: one thread per worker, the caller waits
  std::vector<std::thread>          threads;
  std::mutex                        pool_mutex;
  std::condition_variable           pool_start;
  std::condition_variable           pool_done;
  std::function<void( Worker& )> const * pool_job = nullptr; // valid while pool_pending > 0
  u64                               pool_job_id  = 0;
  int                               pool_pending = 0;
  bool                              pool_quit    = false;

public:
  // envs must outlive the trainer; each is used by exactly one thread. seed: see RANDOM NUMBERS above.
  ParallelPpoTrainer( std::vector<IEnvironment*> const& envs, nn::ActorCriticNet& net, PpoConfig const& config, PolicyKind kind, u64 seed = 1 );
  ~ParallelPpoTrainer();

  ParallelPpoTrainer( ParallelPpoTrainer const& ) = delete;
  ParallelPpoTrainer& operator=( ParallelPpoTrainer const& ) = delete;

  void setMaxStepsInEpisode( int n ) { max_steps_in_episode = n; }

  nn::AdamOptimizer& opt() { return optimizer; }

  // Advances every environment by up to max_steps, never past the end of the rollout.
  // When that fills the rollout, runs the PPO update before returning, and returns true.
  // A renderer calls it with its steps-per-frame; a headless loop calls runIteration().
  bool advance( int max_steps );

  PpoStats runIteration();

  // Ends the current episode of one environment from outside (a key in the viewer, a changed difficulty) and starts a new one.
  void resetEpisode( int env_index );
  void resetAllEpisodes();

  // Returns of the episodes finished since the previous call, all environments, appended to out.
  void takeFinishedEpisodes( std::vector<Real>& out_returns );

  // The mean action of the MASTER net, no sampling. Calling thread only, between advance() calls.
  void actDeterministic( Real const* observation, Real* out_action );

  // --- observability -----------------------------------------------------------------------------
  inline int envsCount  () const { return (int)workers.size(); }
  inline int stepsPerEnv() const { return steps_per_env; }

  PpoStats const& lastStats() const { return last_stats; }

  inline long long totalEnvSteps() const { return total_env_steps; }
  inline int       updatesDone  () const { return updates_done; }

  int rolloutFill() const;                                                      // total over all environments
  inline int rolloutCapacity() const { return steps_per_env * (int)workers.size(); }

  inline Real currentEpisodeReturn( int env_index ) const { return workers[(size_t)env_index]->cur_episode_return; }
  inline int  currentEpisodeLength( int env_index ) const { return workers[(size_t)env_index]->cur_episode_length; }

private:
  void runOnAllWorkers( std::function<void( Worker& )> const& job );
  void threadMain( int index, u64 seed );

  void syncWorkerNet( Worker& w );
  void collect( Worker& w, int steps );
  void cutEpisode( Worker& w );
  void gradientSlice( Worker& w, int const* indices, int count );

  void update();

  u32 shuffleRand( u32 to );
};

} // namespace rl
} // namespace zygo
