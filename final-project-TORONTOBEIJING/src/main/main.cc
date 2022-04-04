#include <iostream>
#include "src/lib/trojanmap.h"
#include "src/lib/mapui.h"
// #define NCURSES

int main() {
  MapUI x;
  #ifdef NCURSES
    x.PlotMap();
    x.DynamicPrintMenu();
  #else
    x.PlotMap();
    x.PrintMenu();
  #endif
  return 0;
}