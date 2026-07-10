#pragma once

#include <zygo/math/common/consts.h>


namespace zygo {

const int MY_SHA160_DATA_SIZE   = 64;
const int MY_SHA160_BLOCK_SIZE  = 20;


class Sha160
{
private:
	u8 data[MY_SHA160_DATA_SIZE];
	u32 datalen;
	u64 bitlen;
	u32 state[8];

public:
	Sha160();

	void update( u8 const input_data[], u32 len );
	void calcFinal( u8 hash[] );

private:
	void transform();
};

} // namespace zygo
