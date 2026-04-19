#ifndef FLOORPLANNER_H
#define FLOORPLANNER_H

#include <fstream>
#include <vector>
#include <map>
#include <string>
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
    double getCost();
    double calcTotalHPWL();
    void perturb();
    void simulatedAnnealing();

    double _alpha;
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
};

#endif
