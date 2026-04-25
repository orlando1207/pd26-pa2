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
    : _alpha(alpha), _timeLimitSec(3600.0), _startClock(0),
      _bestW(0), _bestH(0), _bestCost(DBL_MAX), _bestRoot(-1),
      _normArea(1.0), _normWL(1.0), _tempRatio(1.0)
{
    parseBlock(blkFile);
    parseNet(netFile);
    _tree.randomly_init(_blocks.size());
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

    // ── 1. Normalise area and wirelength to the same scale ──────────────
    double normA  = area / _normArea;
    double normWL = wl   / _normWL;

    // ── 2. Dynamic penalty: loose at high T (explore), tight at low T ──
    //   penaltyCoeff: 1.0 when _tempRatio=1 (hot) → 50.0 when _tempRatio=0 (cold)
    double penaltyCoeff = 1.0 + 49.0 * (1.0 - _tempRatio);
    double penalty = 0;
    if (totalW > _outlineW)
        penalty += (double)(totalW - _outlineW) * (_outlineH + totalH);
    if (totalH > _outlineH)
        penalty += (double)(totalH - _outlineH) * (_outlineW + totalW);

    return _alpha * normA + (1.0 - _alpha) * normWL
           + penaltyCoeff * (penalty / _normArea);
}

void Floorplanner::perturb() {
    int n = _blocks.size();

    // ── Adaptive perturbation weights ────────────────────────────────────
    // rotateP: 10 % at high T  →  40 % at low T  (fine-tuning)
    // The remaining probability is split evenly between delete-insert and swap.
    double rotateP = 0.10 + 0.30 * (1.0 - _tempRatio);
    double moveP   = (1.0 - rotateP) * 0.5;
    double r = (double)rand() / RAND_MAX;
    int op = (r < rotateP) ? 0 : (r < rotateP + moveP) ? 1 : 2;

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

double Floorplanner::autoTuneTemperature() {
    int n = _blocks.size();
    int samples = max(200, 10 * n);

    // Snapshot starting state (shared by both phases below)
    vector<BTreeNode> snapNodes(n);
    int snapRoot;
    _tree.saveState(snapNodes, snapRoot);
    vector<bool> snapRotate(n);
    for (int i = 0; i < n; i++) snapRotate[i] = _blocks[i]->getRotate();

    // Pre-allocate inner-loop buffers
    vector<BTreeNode> tmpNodes(n);
    int tmpRoot;
    vector<bool> tmpRotate(n);

    // ── Phase 1: estimate normalization factors ──────────────────────────
    // Sample raw area and wirelength to establish each term's typical scale.
    // _tempRatio is still 1.0 here, so perturb() uses high-T weights.
    int normSamples = max(50, samples / 4);
    double sumArea = 0, sumWL = 0;
    for (int i = 0; i < normSamples; i++) {
        _tree.saveState(tmpNodes, tmpRoot);
        for (int j = 0; j < n; j++) tmpRotate[j] = _blocks[j]->getRotate();

        perturb();
        int tw, th;
        _tree.pack(_blocks, tw, th);
        sumArea += (double)tw * th;
        sumWL   += calcTotalHPWL();

        _tree.restore(tmpNodes, tmpRoot);
        for (int j = 0; j < n; j++) _blocks[j]->setRotate(tmpRotate[j]);
    }
    _normArea = sumArea / normSamples;
    _normWL   = sumWL   / normSamples;
    if (_normArea < 1.0) _normArea = 1.0;
    if (_normWL   < 1.0) _normWL   = 1.0;

    // ── Phase 2: estimate T0 using the now-calibrated normalised costs ───
    double baseCost = getCost();
    double sumAbsDelta = 0;

    for (int i = 0; i < samples; i++) {
        _tree.saveState(tmpNodes, tmpRoot);
        for (int j = 0; j < n; j++) tmpRotate[j] = _blocks[j]->getRotate();

        perturb();
        double newCost = getCost();
        sumAbsDelta += fabs(newCost - baseCost);

        _tree.restore(tmpNodes, tmpRoot);
        for (int j = 0; j < n; j++) _blocks[j]->setRotate(tmpRotate[j]);
    }

    // Restore starting state
    _tree.restore(snapNodes, snapRoot);
    for (int i = 0; i < n; i++) _blocks[i]->setRotate(snapRotate[i]);

    double avgDelta = sumAbsDelta / samples;
    if (avgDelta < 1e-9) avgDelta = 1e-9;
    // T0 s.t. P(accept bad move) ~= 0.8: T0 = -avgDelta / ln(0.8) = avgDelta / 0.2231
    return avgDelta / 0.2231;
}

void Floorplanner::simulatedAnnealing() {
    _startClock = clock();
    int n = _blocks.size();

    // Auto-tune initial temperature from cost landscape
    double T = autoTuneTemperature();
    double T0 = T;          // stored for _tempRatio computation
    _tempRatio = 1.0;
    double coolingRate = 0.999;
    // Tmin: SA terminates naturally when acceptance probability is negligible.
    // Chosen so that at Tmin the acceptance prob for a 1-sigma bad move is < 1e-4.
    double Tmin = T0 * 1e-4;
    if (Tmin < 1e-12) Tmin = 1e-12;
    int iterPerTemp = max(10 * n, 200);

    double curCost = getCost();
    int curW = _tree.getLastW(), curH = _tree.getLastH();

    // Save initial state as best (even if infeasible)
    _bestCost = curCost;
    _bestW = curW; _bestH = curH;
    _bestNodes.resize(n);
    _tree.saveState(_bestNodes, _bestRoot);
    _bestRotate.resize(n);
    for (int i = 0; i < n; i++) _bestRotate[i] = _blocks[i]->getRotate();

    // Pre-allocate saved-state buffers once (no per-iteration heap allocation)
    vector<BTreeNode> savedNodes(n);
    int savedRoot;
    vector<bool> savedRotate(n);

    while (T > Tmin) {
        _tempRatio = T / T0;
        for (int i = 0; i < iterPerTemp; i++) {
            // Save current state into pre-allocated buffers
            _tree.saveState(savedNodes, savedRoot);
            for (int j = 0; j < n; j++) savedRotate[j] = _blocks[j]->getRotate();

            perturb();
            double newCost = getCost();
            // Retrieve W/H from the pack() already called inside getCost()
            int ntw = _tree.getLastW(), nth = _tree.getLastH();
            double delta = newCost - curCost;

            if (delta < 0 || (double)rand() / RAND_MAX < exp(-delta / T)) {
                curCost = newCost;
                curW = ntw; curH = nth;
                // Update best only for feasible solutions
                if (ntw <= _outlineW && nth <= _outlineH && newCost < _bestCost) {
                    _bestCost = newCost;
                    _bestW = ntw; _bestH = nth;
                    _tree.saveState(_bestNodes, _bestRoot);
                    for (int j = 0; j < n; j++) _bestRotate[j] = _blocks[j]->getRotate();
                }
            } else {
                // Reject: restore from pre-allocated buffers
                _tree.restore(savedNodes, savedRoot);
                for (int j = 0; j < n; j++) _blocks[j]->setRotate(savedRotate[j]);
            }
        }
        if (T > 1e-12) T *= coolingRate;

        // Hard 3600 s safety cutoff
        double elapsed = (double)(clock() - _startClock) / CLOCKS_PER_SEC;
        if (elapsed >= _timeLimitSec) break;
    }

    // Restore best found solution
    _tree.restore(_bestNodes, _bestRoot);
    for (int i = 0; i < n; i++) _blocks[i]->setRotate(_bestRotate[i]);
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
