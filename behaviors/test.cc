#include "test.h"

TestBehavior::
TestBehavior( const std::string teamName,
                  int uNum,
                  const map<string, string>& namedParams_,
                  const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void TestBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    VecPosition space = worldModel->getBall();
    beamX = space.getX() + 5;
    beamY = space.getY() + 5;
    beamAngle = 0;
}

SkillType TestBehavior::
selectSkill() {
    return SKILL_WALK_OMNI;
}