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
    VecPosition space = worldModel->getGoalPost(0);
    beamX = space.getX();
    beamY = space.getY();
    beamAngle = 0;
}

SkillType TestBehavior::
selectSkill() {
    return SKILL_WALK_OMNI;
}