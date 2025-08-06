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
#include <omnetpp.h>
#include "ArrivalChecker.h"
#include "../mobility/DroneLinearMobility.h"
#include "../beaconing/processor/Processor.h"
#include <inet/physicallayer/neighborcache/NeighborListNeighborCache.h>
#include <inet/physicallayer/contract/packetlevel/IRadio.h>
#include <vector>
#include <set>
#include <inet/common/ModuleAccess.h>
using namespace std;

Define_Module(ArrivalChecker);
simsignal_t collisionCountId = cComponent::registerSignal("collisionCount");
simsignal_t allId = cComponent::registerSignal("allFinishNode");
simsignal_t rightId = cComponent::registerSignal("rightFinishNode");
simsignal_t simTimeId = cComponent::registerSignal("simTimeValue");
simsignal_t collisionNodeId = cComponent::registerSignal("collisionNode");
simsignal_t collisionVecId = cComponent::registerSignal("collisionVec");
simsignal_t nodeCountId = cComponent::registerSignal("nodeCount");
simsignal_t lossCountId = cComponent::registerSignal("lossCount");
simsignal_t arriveCountId = cComponent::registerSignal("arriveCount");

void ArrivalChecker::initialize() {
    startTime = clock();
    check = new cMessage("check");
    count = new cMessage("count");
    minimumSafetyDistance = par("minimumSafetyDistance");
    numOfTarget = par("numOfTarget");
    constrainZ = par("constrainZ").doubleValue();
    scheduleAt(simTime()+CL_CHECK_PERIOD, check);
    scheduleAt(simTime(), count);
    currentNumofTarget = 0;
    arriveCount = 0;
}


void ArrivalChecker::handleMessage(cMessage* msg) {
    if (msg == check) {
       checkCollision();
       scheduleAt(simTime()+CL_CHECK_PERIOD, check);
    } else if (msg == count) {
       countNode();
       scheduleAt(simTime()+NODE_COUNT_CHECK_PERIOD, count);
    } else {
        error("ArrivalChecker::handleMessage unknown message");
    }
}

void ArrivalChecker::countNode() {

    int c = 0;
    int loss = 0;
    for (int i=0; i<numOfTarget;i++) {
        cModule *module = getParentModule()->getSubmodule("nodes", i);
        if (module != nullptr) {
            DroneLinearMobility * mobility = check_and_cast<DroneLinearMobility *>(module->getSubmodule("mobility"));
            if (!mobility->isArrival()) {
                c += 1;
            }
            Processor *p = check_and_cast<Processor *>(module->getSubmodule("p"));
            loss += p->getPacketLoss();
        }
    }
    emit(lossCountId, loss);
    emit(nodeCountId, c);
    emit(arriveCountId, arriveCount);
    arriveCount = 0;
}



void ArrivalChecker::checkCollision() {
    set<int> collided;
    MatrixXd positions;
    positions.resize(numOfTarget, 3);
    for (int i=0; i<numOfTarget;i++) {
        cModule *module = getParentModule()->getSubmodule("nodes", i);
        if (module != nullptr) {
            DroneLinearMobility *mobility = check_and_cast<DroneLinearMobility *>(module->getSubmodule("mobility"));
            Coord p = mobility->getCurrentPosition();
            positions.row(i) = Vector3d(p.x, p.y, p.z);
            if (mobility->isArrival()) {
                currentNumofTarget++;
                arriveCount++;
                //if (mobility->getRightFinish()) emit(rightId, true);
                emit(allId, true);
                std::cout << "delete " << i << " at " << simTime() << endl;
                module->callFinish();
                module->deleteModule();
                if (currentNumofTarget == numOfTarget) {
                    endSimulation();
                }
                positions.row(i) = Vector3d(-1,-1,-1);
            }
        } else {
            positions.row(i) = Vector3d(-1,-1,-1);
        }
    }

    for (int i=0; i<numOfTarget;i++) {
        Vector3d p1 = positions.row(i);
        if (!p1.isApprox(Vector3d(-1,-1,-1))) {
            for (int j=i+1;j<numOfTarget;j++){
                Vector3d p2 = positions.row(j);
                if (p2.isApprox(Vector3d(-1,-1,-1))) continue;
                Vector3d disV = p1.array()-p2.array();
                double dis = disV.norm();
                if (dis <= minimumSafetyDistance) {
                    emit(collisionCountId, true);
                    collided.insert(i);
                    collided.insert(j);
                    emit(collisionNodeId,i);
                    emit(collisionNodeId,j);
                    //std::cout << "--------------------------------" << endl;
                    //std::cout << "ArrivalChecker:: node " << i << " and node " << j << " collided and will be removed" << endl;
                    //std::cout << "ArrivalChecker:: --at time " << simTime() << " and distance is " << dis << endl;
                }
            }
        }
    }
    // delete the collided nodes detected above
    set<int>::iterator it;
    for (it=collided.begin();it!=collided.end();it++) {
        cModule *module = getParentModule()->getSubmodule("nodes", *it);
        if (module != nullptr) {
            module->callFinish();
            module->deleteModule();
        }
    }
}


ArrivalChecker::~ArrivalChecker() {
    cancelAndDelete(check);
    cancelAndDelete(count);
}

void ArrivalChecker::finish()
{
    emit(simTimeId, clock()-startTime);
    cSimpleModule::finish();
}
