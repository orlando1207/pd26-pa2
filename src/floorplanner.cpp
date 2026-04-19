#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <climits>
#include <cfloat>
#include "floorplanner.h"
using namespace std;

Floorplanner::Floorplanner(fstream& blkFile, fstream& netFile, double alpha)
    : _alpha(alpha), _bestW(0), _bestH(0), _bestCost(DBL_MAX), _bestRoot(-1)
{
    parseBlock(blkFile);
    parseNet(netFile);
    _tree.init(_blocks.size());
}

Floorplanner::~Floorplanner() {
    for (auto* b : _blocks) delete b;
    for (auto* t : _terminals) delete t;
    for (auto* n : _nets) delete n;
}

void Floorplanner::parseBlock(fstream& input) {
    string line, token;
    // Outline
    input >> token >> _outlineW >> _outlineH;
    int numBlocks, numTerminals;
    input >> token >> numBlocks;
    input >> token >> numTerminals;

    for (int i = 0; i < numBlocks; i++) {
        string name; int w, h;
        input >> name >> w >> h;
        Block* b = new Block(name, w, h);
        _blocks.push_back(b);
        _nameToTerm[name] = b;
    }

    for (int i = 0; i < numTerminals; i++) {
        string name, dummy; int x, y;
        input >> name >> dummy >> x >> y;
        Terminal* t = new Terminal(name, x, y);
        _terminals.push_back(t);
        _nameToTerm[name] = t;
    }
}

void Floorplanner::parseNet(fstream& input) {
    string token;
    int numNets;
    input >> token >> numNets;

    for (int i = 0; i < numNets; i++) {
        int degree;
        input >> token >> degree;
        Net* net = new Net();
        for (int j = 0; j < degree; j++) {
            string name;
            input >> name;
            auto it = _nameToTerm.find(name);
            if (it != _nameToTerm.end())
                net->addTerm(it->second);
        }
        _nets.push_back(net);
    }
}

double Floorplanner::calcTotalHPWL() {
    double total = 0;
    for (auto* n : _nets)
        total += n->calcHPWL();
    return total;
}

double Floorplanner::getCost() {
    int totalW, totalH;
    _tree.pack(_blocks, totalW, totalH);

    double area = (double)totalW * totalH;
    double wl = calcTotalHPWL();

    // Penalty for exceeding outline
    double penalty = 0;
    if (totalW > _outlineW)
        penalty += (totalW - _outlineW) * (_outlineH + totalH);
    if (totalH > _outlineH)
        penalty += (totalH - _outlineH) * (_outlineW + totalW);

    return _alpha * area + (1.0 - _alpha) * wl + 10.0 * penalty;
}

void Floorplanner::perturb() {
    int n = _blocks.size();
    int op = rand() % 3;

    if (op == 0) {
        // Rotate a block
        int idx = rand() % n;
        int bIdx = _tree.getNodes()[idx].blockIdx;
        _blocks[bIdx]->setRotate(!_blocks[bIdx]->getRotate());
    } else if (op == 1) {
        // Delete and insert
        int delIdx = rand() % n;
        int insIdx = rand() % n;
        while (insIdx == delIdx) insIdx = rand() % n;
        bool toLeft = rand() % 2;
        _tree.deleteAndInsert(delIdx, insIdx, toLeft);
    } else {
        // Swap two nodes
        int a = rand() % n;
        int b = rand() % n;
        while (b == a) b = rand() % n;
        _tree.swapNodes(a, b);
    }
}

void Floorplanner::simulatedAnnealing() {
    int n = _blocks.size();
    double T = 1e6;
    double coolingRate = 0.999;
    double Tmin = 0.01;
    int iterPerTemp = max(10 * n, 200);

    double curCost = getCost();

    // Save initial as best
    {
        int tw, th;
        _tree.pack(_blocks, tw, th);
        _bestCost = curCost;
        _bestW = tw; _bestH = th;
        _bestNodes = _tree.saveNodes();
        _bestRoot = _tree.saveRoot();
        _bestRotate.resize(n);
        for (int i = 0; i < n; i++)
            _bestRotate[i] = _blocks[i]->getRotate();
    }

    while (T > Tmin) {
        for (int i = 0; i < iterPerTemp; i++) {
            // Save state
            auto savedNodes = _tree.saveNodes();
            int savedRoot = _tree.saveRoot();
            vector<bool> savedRotate(n);
            for (int j = 0; j < n; j++)
                savedRotate[j] = _blocks[j]->getRotate();

            perturb();
            double newCost = getCost();
            double delta = newCost - curCost;

            if (delta < 0 || (double)rand() / RAND_MAX < exp(-delta / T)) {
                curCost = newCost;
                // Check if this is the best feasible solution
                int tw, th;
                _tree.pack(_blocks, tw, th);
                if (tw <= _outlineW && th <= _outlineH && newCost < _bestCost) {
                    _bestCost = newCost;
                    _bestW = tw; _bestH = th;
                    _bestNodes = _tree.saveNodes();
                    _bestRoot = _tree.saveRoot();
                    for (int j = 0; j < n; j++)
                        _bestRotate[j] = _blocks[j]->getRotate();
                }
            } else {
                // Reject: restore
                _tree.restore(savedNodes, savedRoot);
                for (int j = 0; j < n; j++)
                    _blocks[j]->setRotate(savedRotate[j]);
            }
        }
        T *= coolingRate;
    }

    // Restore best
    _tree.restore(_bestNodes, _bestRoot);
    for (int i = 0; i < n; i++)
        _blocks[i]->setRotate(_bestRotate[i]);
    int tw, th;
    _tree.pack(_blocks, tw, th);
    _bestW = tw; _bestH = th;
}

void Floorplanner::floorplan() {
    simulatedAnnealing();
}

void Floorplanner::writeResult(fstream& output, double runtime) {
    int tw, th;
    _tree.pack(_blocks, tw, th);
    double area = (double)tw * th;
    double wl = calcTotalHPWL();
    double cost = _alpha * area + (1.0 - _alpha) * wl;

    output << (int)round(cost) << endl;
    output << (int)round(wl) << endl;
    output << (int)round(area) << endl;
    output << tw << " " << th << endl;
    output << fixed;
    output.precision(6);
    output << runtime << endl;

    for (auto* b : _blocks) {
        output << b->getName() << " "
               << b->getX1() << " " << b->getY1() << " "
               << b->getX2() << " " << b->getY2() << endl;
    }
}
