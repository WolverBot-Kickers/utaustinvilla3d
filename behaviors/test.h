#ifndef _TEST_H
#define _TEST_H

#include "naobehavior.h"


class TestBehavior : public NaoBehavior {

    TestBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);

    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual SkillType selectSkill();

};



#endif