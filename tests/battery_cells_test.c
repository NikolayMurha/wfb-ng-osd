#include "battery_cells.h"
#include <assert.h>
#include <math.h>
int main(void) {
    uint16_t cells[10], ext[4] = {0};
    for (int i=0; i<10; ++i) cells[i] = UINT16_MAX;
    cells[0] = 24000;
    assert(battery_cell_average(cells, ext, 0) == 0);
    assert(fabs(battery_cell_average(cells, ext, 6) - 4) < .001);
    cells[0] = 4100; cells[1] = 4000; cells[2] = 4050; cells[3] = 4050;
    assert(fabs(battery_cell_average(cells, ext, 0) - 4.05) < .001);
    cells[1] = UINT16_MAX;
    assert(battery_cell_average(cells, ext, 0) == 0);
    for (int i=0; i<10; ++i) cells[i] = 4000;
    ext[0] = ext[1] = 4000;
    assert(fabs(battery_cell_average(cells, ext, 0) - 4) < .001);
    for (int i=0; i<10; ++i) cells[i] = UINT16_MAX;
    ext[0] = ext[1] = 0;
    cells[0] = 65534; cells[1] = 14466;
    assert(battery_cell_average(cells, ext, 0) == 0);
    assert(fabs(battery_cell_average(cells, ext, 20) - 4) < .001);
}
