#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <ctime>
#include "module.h"
#include "btree.h"

class Floorplanner {
public:
    Floorplanner(fstream& blkFile, fstream& netFile, double alpha);
    ~Floorplanner();

    void floorplan();
    void writeResult(fstream& output, double runtime);

private:
    void parseBlock(fstream& input);
    void parseNet(fstream& input);
    double calcTotalHPWL();
    double getCost();
    void perturb();
    double autoTuneTemperature();
    void simulatedAnnealing();

    double _alpha;
    double _timeLimitSec;
    clock_t _startClock;
    int _outlineW, _outlineH;

    vector<Block*> _blocks;
    vector<Terminal*> _terminals;
    vector<Net*> _nets;
    map<string, Terminal*> _nameToTerm;

    BTree _tree;

    // Best solution
    int _bestW, _bestH;
    double _bestCost;
    vector<BTreeNode> _bestNodes;
    int _bestRoot;
    vector<bool> _bestRotate;

    // SA adaptive state
    double _normArea;    // avg area used to normalise cost
    double _normWL;      // avg wirelength used to normalise cost
    double _tempRatio;   // current T / T0  (1.0 = hot, 0.0 = cold)
};

#endif
