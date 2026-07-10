#include "windows_funcs.h"


std::string chooseFileToOpen( HWND owner_window_hwnd )
{
  const int SYMBOLS_MAX_COUNT = 260;
  const int BUF_SIZE          = SYMBOLS_MAX_COUNT * 2 + 1;

  OPENFILENAME ofn;     // common dialog box structure
  WCHAR buf[BUF_SIZE];  // buffer for file name

  // Initialize OPENFILENAME
  ZeroMemory( &ofn, sizeof( ofn ) );
  ofn.lStructSize = sizeof( ofn );
  ofn.hwndOwner = owner_window_hwnd;
  ofn.lpstrFile = buf;

  // Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
  // use the contents of buf to initialize itself.
  ofn.lpstrFile[0]    = '\0';
  ofn.nMaxFile        = SYMBOLS_MAX_COUNT;

  ofn.lpstrFilter     = L"All\0*.*\0Data\0*.dat\0";
  ofn.nFilterIndex    = 1;
  ofn.lpstrFileTitle  = NULL;
  ofn.nMaxFileTitle   = 0;
  ofn.lpstrInitialDir = NULL;
  ofn.Flags           = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

  // Display the Open dialog box. 
  if ( GetOpenFileName( &ofn ) != TRUE )
    return std::string();

  int len = lstrlen( buf );
  int len2 = len * 2 + 2;
  char* str = new char[len2];
  size_t len_converted = len;
  wcstombs_s( &len_converted, str, len2, buf, len2 );

  std::string res( str );
  delete[] str;

  return res;
}

