//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#include "Processor.h"
#include <inet/common/InitStages.h>
#include "../neighbortable/NeighborTable.h"
#include <inet/common/packet/Packet.h>
#include "../../lbp/LocalBroadcastProtocol.h"
#include <inet/common/ModuleAccess.h>
#include "../../mobility/MovingInfo_m.h"

Define_Module(Processor);
simsignal_t countId = cComponent::registerSignal("overtakeCount");
simsignal_t perferredSpeedId = cComponent::registerSignal("perferredSpeedValue");
simsignal_t speedCostId = cComponent::registerSignal("speedCost");
simsignal_t switchCostId = cComponent::registerSignal("switchCost");
simsignal_t positionCostId = cComponent::registerSignal("positionCost");
simsignal_t totalCostId = cComponent::registerSignal("totalCost");


using namespace inet;

Processor::Processor ()
{
}

void Processor::finish()
{
}

Processor::~Processor()
{
    cancelAndDelete(startPlan);
}

// ----------------------------------------------------


void Processor::initialize (int stage)
{
  if (stage == INITSTAGE_LOCAL)
    {
      gidBeaconsIn = findGate("beaconsIn");
      gateToM = gate("toM");
      gateMIn = gate("mIn");

      minimumSafetyDistance = par("minimumSafetyDistance");
      maxNeighborRange      = par("maxNeighborRange");
      clTimeThreshold       = par("clTimeThreshold");
      overtakeSpeed         = par("overtakeSpeed");
      otherLaneSafetyRange  = par("E2");
      targetLaneSafetyRange = par("E1");


      KAPPA_Speed = par("K2");
      KAPPA_Position = par("K1");
      KAPPA_Switch = par("K3");
      KAPPA_SINTERVAL = par("KI");
      E_SPEED = par("E3");
      E_TARGET = par("E2");
      E_NEIGHBOR = par("E3");

      startPlan             = new cMessage("startPlan");

      fisrtPart         = par("fisrtPart").stringValue();
      endPart           = par("endPart").stringValue();
      currentPart       = fisrtPart;
      currentI          = par("currentI");
      currentJ          = par("currentJ");
    }

  if (stage == INITSTAGE_LAST)
  {
      cModule *host = getContainingNode(this);
      assert(host);
      LocalBroadcastProtocol *lbp = check_and_cast<LocalBroadcastProtocol*>(host->getSubmodule("lbp"));
      assert(lbp);
      ownIdentifier  =  lbp->getOwnMacAddress();
      rb = check_and_cast<RenewalBeaconing*>(host->getSubmodule("rb"));
      mobility  = check_and_cast<DroneLinearMobility*>(host->getSubmodule("mobility"));
      nbTable = check_and_cast<NeighborTable*>(host->getSubmodule("nbt"));
      assert(mobility);
      cGate* destIn = mobility->gate("pIn");
      cGate* destOut = mobility->gate("toP");
      gateToM->connectTo(destIn);
      destOut->connectTo(gateMIn);

      preferredSpeed = rb->getPreferredSpeedd();
      maxSpeed = rb->getMaxSpeed();
      emit(perferredSpeedId, preferredSpeed);
      rs = check_and_cast<RoadSystem *>(getParentModule()->getParentModule()->getSubmodule("roadSystem"));
      rp = rs->getRoadPath(fisrtPart, endPart);
      setNextNextTarget();
      scheduleAt(simTime()+1.3, startPlan);
  }
}

void Processor::moveToNextPart() {
    Lane next = rp.getToLane();
    std::string _id = rp.getNextId();
    currentPart = _id;
    double _t = rp.getToT();
    int totalt = rs->getLaneCoord(Lane(0, 0), _id).rows()-1;
    totalt = round(totalt*_t);
    Vector3d nextT = rs->getWayPoint(totalt+1, _id, next);
    mobility->setNexTPart(_id, totalt, next.getI(), next.getJ());
    rp.toNextPoint();
    setNextNextTarget();
}



void Processor::setNextNextTarget() {
    if (currentPart == endPart) {
        mobility->takeNewTarget(Vector3d(0,0,0), 9999, 9999);
        return;
    }
    Lane next = rp.getToLane();
    std::string _id = rp.getNextId();
    double _t = rp.getToT();
    int totalt = rs->getLaneCoord(Lane(0, 0), _id).rows()-1;
    totalt = round(totalt*_t);
    double _t1 = rp.getFromT();
    int totalt2 = round(rs->getLaneCoord(Lane(0, 0), currentPart).rows() * _t1);;
    Vector3d nextT = rs->getWayPoint(totalt+1, _id, next);
    mobility->takeNewTarget(nextT, totalt2, _t1);
}

AlgorithmOutput Processor::findNextLane() {

    //bool print = false;
    //if ((ownIdentifier == MacAddress("0AAA00000593") or ownIdentifier == MacAddress("0AAA00000000")) and simTime() > 780) print = false;

    double t = mobility->getCurrentT();
    std::vector<Lane> allNeighbors = rs->getNeighborLane(Lane(currentI, currentJ), currentPart, t, 1);
    allNeighbors.push_back(Lane(currentI, currentJ));
    std::vector<double> allCosts;
    std::vector<Lane> nextLs;
    std::vector<double> speedDiffs;
    AlgorithmOutput output;
    double min = MAX;
    EV << "\nProcessor::runAlgorithm: Start printing cost:\n";
    EV << "Lane          speed          collision          Position      Total\n";

    //if (print) {
    //    std::cout << "==========================================--------------================================" << endl;
    //    std::cout << ownIdentifier << "    target: " << rp.getFromT() <<endl;
    //    std::cout << currentPart << "   "  << mobility->getCurrentT() << "   "  << currentI <<", " << currentJ << endl;
    //}

    for(auto&i : allNeighbors) {
        double speedCost = getSpeedCost(i);
        double collisionCost = getCollisionCost(i);
        double positionCost = getPositionCost(i);

        // define f(dT)
        double coeff_p = 1;
        if (rp.getFromT() > 0) {
            double disToTarget = -1;
            disToTarget = rs->getDistance(currentPart, Lane(currentI, currentJ), t, rp.getFromT());
            if (disToTarget <= 15) coeff_p = 20.0;
            else if (disToTarget <= 30) coeff_p = (-2.0/3.0)*disToTarget+30.0;
            else coeff_p = 10.0 * 30.0 / disToTarget;
        }
        int positionCoeff = coeff_p * KAPPA_Position;
        // end define

        double totalCost = KAPPA_Speed * speedCost*speedCost + KAPPA_Switch * collisionCost + positionCoeff * positionCost;
        EV << "Lane (" << i.getI() << ", " << i.getJ() << ")   ";
        EV << speedCost << "        " << collisionCost << "        " << positionCost << "       "    << totalCost << endl;
        //if (print) {
        //    std::cout << "Lane (" << i.getI() << ", " << i.getJ() << ")   ";
        //    std::cout << speedCost << "        " << collisionCost << "        " << positionCost << "       "    << totalCost << endl;
        //}
        allCosts.push_back(totalCost);
        speedDiffs.push_back(speedCost);
        if (i == Lane(currentI, currentJ)) {
            output.l = i;
            output.speed = preferredSpeed - speedCost;
        }
        if (totalCost < min) {
            min = totalCost;
        }
    }
    std::vector<double> _speedDiffs;
    for(int i=0;i<allNeighbors.size();i++) {
        if (allCosts[i] == min) {
            nextLs.push_back(allNeighbors[i]);
            _speedDiffs.push_back(speedDiffs[i]);
        }
    }
    if (nextLs.size() == 0 or min > 10000) {
        output.speed = 0;
        return output;
    }
    if (std::find(nextLs.begin(), nextLs.end(), Lane(currentI, currentJ)) != nextLs.end()) return output;
    int randomIndex = floor(par("random01").doubleValue()*nextLs.size());
    output.speed = preferredSpeed - _speedDiffs[randomIndex];
    output.l = nextLs[randomIndex];
    return output;
}

void Processor::makeDecision(AlgorithmOutput output) {
    Lane newLane = output.l;
    double newSpeed = output.speed;
    if (newLane == Lane(currentI, currentJ) and newLane != rp.getFromLane()) {
        //-----------------------------------------------------------------------------------
        if (rs->checkIsTurnLane(currentPart, newLane)) {
            double closeT = rs->getRoadClose(currentPart, newLane, mobility->getCurrentT(), 1);
            double dis2 = rs->getDistance(currentPart, newLane, mobility->getCurrentT(), closeT);
            if (dis2 < newSpeed*KAPPA_STOP) newSpeed = 0;
            EV << "Stop before the inter" << endl;
            EV << "Distance: " << dis2 << endl;
            EV << "T: " << mobility->getCurrentT() << endl;
            EV << "targetT: " << closeT << endl;
        //-----------------------------------------------------------------------------------
        } else {
            double dis2 = -1;
            if (rp.getFromT() > 0) dis2 = rs->getDistance(currentPart, newLane, mobility->getCurrentT(), rp.getFromT());
            double maxMargin = preferredSpeed*KAPPA_STOP;
            if (dis2 < maxMargin and dis2 > 0) newSpeed = 0;
            EV << "Distance: " << dis2 << endl;
            EV << "T: " << mobility->getCurrentT() << endl;
            EV << "targetT: " << rp.getFromT() << endl;
  //---------------------------------------------------------
  //    an approach to increase priority, that will not affect
  //    any metrics but can avoid some drone wait too long
  //---------------------------------------------------------
          if (waitTime > 5 and couldMove and newSpeed == 0) {
              if (dis2 > 3 and dis2 < maxMargin) {
                  dis2 -= 2;
                  newSpeed = dis2/2.0;
                  if (newSpeed > preferredSpeed/2.0) newSpeed = preferredSpeed/2.0;
              }
          }
        }
    }
    if (newSpeed != 0) {
        waitTime = 0;
        lastRun = 0;
    }
    else {
        if (lastRun == 0) lastRun = simTime();
        else {
            waitTime += (simTime() - lastRun).dbl();
            lastRun = simTime();
        }
    }
    //--------------------------------------------------------

    mobility->setSpeed(newSpeed);
    if (newLane != Lane(currentI, currentJ) and newSpeed != 0) {
        laneSwitchTo = newLane;
        double upper = rs->getRoadClose(currentPart, laneSwitchTo, mobility->getCurrentT());
        double targetUpper = rp.getFromT();
        if (upper > targetUpper and targetUpper > 0) upper = targetUpper;
        EV << "Upper: " << upper << endl;
        mobility->switchLane(laneSwitchTo, newSpeed, upper);
    }
}


void Processor::runAlgorithm() {

    if (mobility->isArrival()) return;

    if (mode == "blind") {
        return; // Abandon part
    } else {
        couldMove = true;
        getCandidateNodes();
        EV << "--------------------------------------------------------------------------------------\n";
        EV << "Processor::runAlgorithm: Start printing candidate nodes:\n";
        EV << "ID:" << ownIdentifier << endl;
        EV << "Current Lane (" << currentI << ", " << currentJ << ")     Target Lane (" <<rp.getFromLane().getI() << ","
                << rp.getFromLane().getJ() << ")\n";
        EV << "Current Part: " << currentPart << endl;
        EV << "---------------------In neighbor lane-----------------------\n";
        for(auto l: neighborNodes) {
            Lane lane = l.first;
            EV << "In Lane (" << lane.getI() << ", " << lane.getJ() << "): ";
            for (auto n : l.second) EV << n << ", ";
            EV << "\n";
        }
        EV << "----------------In neighbor neighbor lane--------------------\n";
        for(auto l: nNeighborNodes) {
                Lane lane = l.first;
                EV << "In Lane (" << lane.getI() << ", " << lane.getJ() << "): ";
                for (auto n : l.second) EV << n << "--" << (*nbTable)[n].getDistance() << ", ";
                EV << "\n";
        }
        AlgorithmOutput output = findNextLane();
        if ((ownIdentifier == MacAddress("0AAA00000593") or ownIdentifier == MacAddress("0AAA00000000")) and simTime() > 780) {
            std::cout << "---------------0" << endl;
            std::cout << mobility->getCurrentT() << endl;
        }
        EV << "----------------Algorithm Output--------------------\n";
        EV << "Lane: (" << output.l.getI() << ", " << output.l.getJ() << ")" << endl;
        EV << "Speed: " << output.speed << endl;

        EV << "-----------------To next target-----------------------\n";
        makeDecision(output);
        clearCandidateNodes();
        EV <<"current speed: "  << mobility->getMaxSpeed() << endl;

        if (rs->getNeighborLane(Lane(currentI, currentJ), currentPart, mobility->getCurrentT()).size() == 0) scheduleAt(simTime()+KAPPA_SINTERVAL, startPlan);
        else if (mobility->getMaxSpeed()  == 0) scheduleAt(simTime()+KAPPA_SINTERVAL, startPlan);
        else scheduleAt(simTime()+2, startPlan);
        EV << "--------------------------------------------------------------------------------------\n";
    }
}


// ----------------------------------------------------


void Processor::handleMovingInfo(MovingInfo* mi) {

    EV << "receive type: " << mi->getType() << endl;
    if (mi->getType()==1) {
        currentI = laneSwitchTo.getI();
        currentJ = laneSwitchTo.getJ();
    } else if (mi->getType()==0) {
        currentI = rp.getToLane().getI();
        currentJ = rp.getToLane().getJ();
        moveToNextPart();
        cancelEvent(startPlan);
        scheduleAt(simTime(), startPlan);
    } else {
        cancelEvent(startPlan);
        scheduleAt(simTime(), startPlan);
    }
}

// ------------------------------------------------------------------------------------------------------------------------------------

int Processor::getPacketLoss()
{
    int loss = 0;
    for (auto it = nblostTable.begin(); it != nblostTable.end(); ++it)
    {
        auto key = it->first;
        loss += it->second;
        nblostTable[key] = 0;
    }
    return loss;
}

void Processor::updateClTime(MacAddress nb) {
    int itI                = (*nbTable)[nb].getLastI();
    int itJ                = (*nbTable)[nb].getLastJ();
    std::string itRoadId   = (*nbTable)[nb].getLastRoadId();
    if (itRoadId == currentPart) {
        // update distance
        double itT             = (*nbTable)[nb].getLastT();
        double dis;
        if (itT >= mobility->getCurrentT()) dis = rs->getDistance(itRoadId, Lane(itI,itJ), mobility->getCurrentT(), itT);
        else dis = -1 * rs->getDistance(itRoadId, Lane(itI,itJ), itT, mobility->getCurrentT());
        (*nbTable)[nb].setDistance(dis);
        // update clTime
        if (itI == mobility->getCurrI() and itJ == mobility->getCurrJ()) {
            double speedDiff;
            double clTime = -1;
            if (itT >= mobility->getCurrentT()) speedDiff = mobility->getMaxSpeed() - (*nbTable)[nb].getLastVelocity().length();
            else speedDiff = (*nbTable)[nb].getLastVelocity().length() - mobility->getMaxSpeed();
            if (speedDiff > 0.00001) clTime = abs(dis) / abs(speedDiff);
            if (clTime >= 1000) clTime = -1;
            (*nbTable)[nb].setEstCollsionTime(clTime);
            if (dis>0 and clTime < 1 and clTime > 0 and !mobility->getIsSwitching()) {
                EV <<"  stop" << endl;
                EV << (*nbTable)[nb].getLastVelocity().length() << "   "  << mobility->getMaxSpeed() << endl;
                EV << nb << "     "  << clTime << "   "  << dis << endl;
                mobility->setSpeed(0);
            }
        } else {
            (*nbTable)[nb].setEstCollsionTime(-1);
        }
    }
    else {
        int isConnect = rs->checkConnect(currentPart, itRoadId, mobility->getCurrI(), itI, mobility->getCurrJ(), itJ);
        double fT, bT, fS, bS, cFT, cTT;
        std::string fId, bId;
        int fi,fj,bi,bj;
        double clTime = -1;
        if (isConnect == 1) {
            fT = mobility->getCurrentT();
            bT = (*nbTable)[nb].getLastT();
            fId = currentPart;bId = itRoadId;
            fi = mobility->getCurrI();fj = mobility->getCurrJ();
            bi = itI;bj = itJ;
            fS = mobility->getMaxSpeed();
            bS = (*nbTable)[nb].getLastVelocity().length();
        } else if (isConnect == 2) {
            bT = mobility->getCurrentT();
            fT = (*nbTable)[nb].getLastT();
            bId = currentPart;fId = itRoadId;
            bi = mobility->getCurrI();bj = mobility->getCurrJ();
            fi = itI;fj = itJ;
            bS = mobility->getMaxSpeed();
            fS = (*nbTable)[nb].getLastVelocity().length();
        } else {
            if (rs->checkAConnectToB(currentPart, itRoadId)) {
                fT = mobility->getCurrentT();
                bT = (*nbTable)[nb].getLastT();
                fId = currentPart;bId = itRoadId;
                fi = mobility->getCurrI();fj = mobility->getCurrJ();
                bi = itI;bj = itJ;
                fS = mobility->getMaxSpeed();
                bS = (*nbTable)[nb].getLastVelocity().length();
            } else {
                (*nbTable)[nb].setDistance(-99999);
                (*nbTable)[nb].setEstCollsionTime(clTime);
                return;
            }
        }
        auto ts = rs->getConnectT(fId, bId);
        cFT = ts[0];
        cTT = ts[1];
        double dis1 = rs->getDistance(fId, Lane(fi,fj), fT, cFT);

        if (abs(dis1) >= 30) {
            (*nbTable)[nb].setDistance(-99999);
            (*nbTable)[nb].setEstCollsionTime(clTime);
            return;
        }

        double speedDiff = fS - bS;
        double dis2;
        if (bT >= cTT) {
            dis2 = rs->getDistance(bId, Lane(bi,bj), cTT, bT);
            (*nbTable)[nb].setDistance(dis1+dis2);
            if (speedDiff > 0.00001) clTime = abs(dis1+dis2) / abs(speedDiff);
            if (clTime >= 1000) clTime = -1;

            if (isConnect==1 and dis1+dis2>0 and clTime < 1 and clTime > 0 and !mobility->getIsSwitching()) {
                EV <<"  stop" << endl;
                EV << (*nbTable)[nb].getLastVelocity().length() << "   "  << mobility->getMaxSpeed() << endl;
                EV << nb << "     "  << clTime << "   "  <<dis1+dis2 << endl;
                mobility->setSpeed(0);
            }
        } else {
            dis2 = rs->getDistance(bId, Lane(bi,bj), bT, cTT);
            (*nbTable)[nb].setDistance(abs(dis2-dis1));
            if ((dis1 >= dis2 and speedDiff > 0) or (dis1 <= dis2 and speedDiff < 0)) clTime = abs(dis2-dis1) / abs(speedDiff);
            if (clTime >= 1000) clTime = -1;
        }
        (*nbTable)[nb].setEstCollsionTime(clTime);
    }
}

double Processor::getSpeedCost(Lane l) {
    double t = mobility->getCurrentT();
    double cost = 0;
    double minDis = MAX;

    bool isCurrent = false;
    if (l == Lane(currentI, currentJ)) isCurrent = true;

    for(auto&it : neighborNodes[l]) {
        double otherT = (*nbTable)[it].getLastT();
        double distance = (*nbTable)[it].getDistance();
        if (distance < minDis and distance > 0) {
            minDis = distance;
            if (distance <= E_SPEED) cost = preferredSpeed - (*nbTable)[it].getLastVelocity().length();   // E_SPEED = 30
            if (distance <= E_SPEED and isCurrent) couldMove = false;
            double itD = rs->getDistance(currentPart, Lane(currentI, currentJ), otherT, rp.getFromT());
            if (!isCurrent and itD >0 and itD < 1) couldMove = false;
            if (cost < 0 ) cost = 0;
        }
    }
    return cost;
}


double Processor::getCollisionCost(Lane l) {
    double t = mobility->getCurrentT();
    double inter = rs->getRoadInter(currentPart, Lane(currentI, currentJ), t);
    if (l == Lane(currentI, currentJ)) return 0;
    if (neighborNodes[l].size() != 0) {
        for (auto &nb: neighborNodes[l]) {
            if (inter > 0 and l==rp.getFromLane() and (*nbTable)[nb].getLastT() < inter) continue;
            double itDis = (*nbTable)[nb].getDistance();
            if (itDis <= E_TARGET and itDis > -1 * E_TARGET) return MAX; // E_TARGET = 16
        }
    }
    double cost = 0;
    bool isFirst = true;
    double mySpeed = mobility->getMaxSpeed();
    if (mode == "simple") {
        auto nnls = rs->getNeighborLane(l, currentPart, t);
        for (auto ll: nnls) {
            if (nNeighborNodes[ll].size() != 0) {
                    for (auto &nb: nNeighborNodes[ll]) {
                        if (abs((*nbTable)[nb].getDistance()) <= E_NEIGHBOR) {
                            cost = cost + 1;
                            double itT = (*nbTable)[nb].getLastT();
                            if (t < itT ) { isFirst = false; }
                            else if (t== itT and ownIdentifier < nb) isFirst = false;
                            if (abs((*nbTable)[nb].getDistance()) <= 3 and mySpeed < (*nbTable)[nb].getLastVelocity().length()) isFirst = false;

                        }
                    }
            }
        }
    }
    if (isFirst) cost = 0;
    return cost;
}


double Processor::getPositionCost(Lane l) {
    double closeT = rs->getRoadClose(currentPart, l, mobility->getCurrentT());
    if (closeT!=1.0 and (rp.getFromLane() != l or rp.getFromT() > closeT)) {
        double closeDistance = rs->getDistance(currentPart, Lane(mobility->getCurrI(), mobility->getCurrJ()), mobility->getCurrentT(), closeT);
        if (closeDistance < preferredSpeed * 2) {
            return MAX;
        }
    }
    // check whether this lane is a non-target turning lane
    bool isEnd = (currentPart == endPart);
    if (rs->checkIsTurnLane(currentPart, l))
    {
        if (rp.getFromLane() == l) return 0;
        return MAX;
    }
    // other cases
    if (mode == "simple") {
        double dis = rs->getDistance(currentPart, Lane(currentI, currentJ), mobility->getCurrentT(),rp.getFromT());
        int min = 9999;
        for (auto _l : rs->getAllSwitchLane(currentPart, mobility->getCurrentT())) {
            int ldis = LaneUtil::getDistance(_l, l);
            if (ldis < min) min = ldis;
        }
        min = 4-min; // 4 is magic
        if (min <0 ) min = 0;
        if (isEnd) { // do not have next turning lane,
            return min;
        } else {
            return LaneUtil::getDistance(l, rp.getFromLane());
        }
    }
    return 0;

}
void Processor::recordBeacon (BeaconReport* beacon)
{
  assert(beacon);
  MacAddress  id      = beacon->getSenderId();
  uint32_t    seqno   = beacon->getSeqno();
  if (nbTable->entryAvailable(id)) {
      if (seqno - nbSeqTable[id] > 1) {
          nblostTable[id] += (seqno - nbSeqTable[id] -1);
      }
      nbSeqTable[id] = seqno;
  } else {
      nbSeqTable[id] = seqno;
      nblostTable[id] = 0;
  }
  nbTable->recordBeacon(beacon);
  updateClTime(id);
}

void Processor::getCandidateNodes() {
    double t = mobility->getCurrentT();
    std::vector<MacAddress> emptyVector;
    std::vector<Lane> allNeighborLanes;

    allNeighborLanes = rs->getNeighborLane(Lane(currentI, currentJ), currentPart, t);

    for(auto &l : allNeighborLanes) {
        neighborNodes[l] = emptyVector;
        std::vector<Lane> allNNeighborLanes = rs->getNeighborLane(l, currentPart, t);
        for(auto &lll : allNNeighborLanes) nNeighborNodes[lll] = emptyVector;
    }
    neighborNodes[Lane(currentI,currentJ)] = emptyVector;
    for (auto & it : nbTable->getNeighborIdList()) {
        Lane itLane = Lane((*nbTable)[it].getLastI(), (*nbTable)[it].getLastJ());
        std::string itPart = (*nbTable)[it].getLastRoadId();
        if(neighborNodes.find(itLane)!=neighborNodes.end() and itPart==currentPart) neighborNodes[itLane].push_back(it);
        if(nNeighborNodes.find(itLane)!=nNeighborNodes.end() and itPart==currentPart) nNeighborNodes[itLane].push_back(it);
        if (rs->checkConnect(currentPart, itPart, currentI, itLane.getI(), currentJ, itLane.getJ()) == 1) {
            neighborNodes[Lane(currentI, currentJ)].push_back(it);
        }
        if (itPart == rp.getNextId() and rs->getPartType(itPart)== "ramp") {
            if (neighborNodes.find(rp.getFromLane())!= neighborNodes.end()) neighborNodes[rp.getFromLane()].push_back(it);
        }
    }
}

void Processor::clearCandidateNodes() {
    neighborNodes.clear();
    nNeighborNodes.clear();
}

void Processor::handleMessage(cMessage *msg)
{
  if (dynamic_cast<BeaconReport*>(msg) && msg->arrivedOn(gidBeaconsIn))
  {
      BeaconReport* beacon = (BeaconReport*) msg;
      recordBeacon(&(*beacon));
      delete msg;
      EV << mobility->getCurrentT()<< endl;
      return;
  } else if (msg == startPlan)
  {
      runAlgorithm();
      return;
  } else if (dynamic_cast<MovingInfo*>(msg)) {
      MovingInfo* mi = (MovingInfo*) msg;
      handleMovingInfo(mi);
      delete msg;
      return;
  }
  EV << "Processor::handleMessage: got improper message type" << endl;
  endSimulation();
}


void Processor::printTable()
{
    EV << "Processor::handleMessage: Starting print Table for node "<< ownIdentifier << endl;
    EV << "----------------------------------------------------------------------------------" << endl;
    EV << "Address              Position                   Velocity              Estimate Collision time        i,j     roadID      t       distance"       << endl;
    auto listNeighbor = nbTable->getNeighborIdList();
    for (auto nb : listNeighbor)
    {
        EV << nb << "    "
           << (*nbTable)[nb].getLastPosition() << "    "
           << (*nbTable)[nb].getLastVelocity() << "       "
           << (*nbTable)[nb].getEstCollsionTime() << "     " <<
           (*nbTable)[nb].getLastI() <<"," << (*nbTable)[nb].getLastJ() << "  " <<
           (*nbTable)[nb].getLastRoadId() <<"    "<< (*nbTable)[nb].getLastT() <<
           "      " << (*nbTable)[nb].getDistance() << endl;
    }
    EV << "Processor::handleMessage: Printing finished"<< endl;
}

