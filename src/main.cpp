#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "floorplanner.h"
using namespace std;

int main(int argc, char** argv)
{
    if (argc != 5) {
        cerr << "Usage: ./fp <alpha> <input.block> <input.net> <output>" << endl;
        return 1;
    }

    srand(time(NULL));

    double alpha = stod(argv[1]);
    fstream blkFile(argv[2], ios::in);
    fstream netFile(argv[3], ios::in);
    fstream output(argv[4], ios::out);

    if (!blkFile) { cerr << "Cannot open " << argv[2] << endl; return 1; }
    if (!netFile) { cerr << "Cannot open " << argv[3] << endl; return 1; }
    if (!output)  { cerr << "Cannot open " << argv[4] << endl; return 1; }

    auto wallStart = chrono::steady_clock::now();

    Floorplanner* fp = new Floorplanner(blkFile, netFile, alpha);
    fp->floorplan();

    double runtime = chrono::duration<double>(
        chrono::steady_clock::now() - wallStart).count();
    fp->writeResult(output, runtime);

    delete fp;
    return 0;
}
