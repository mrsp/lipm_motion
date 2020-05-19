#include "RobotParameters.h"

#include <stdio.h>
#include <stdlib.h>

float RobotParameters::getWalkParameter(int p)
{
    if(p>ParametersSize || p < 0)
    {
        std::cerr << "Error Invalid parameter" << std::endl;
        return 0;
    }
    return WalkParameters[p];
}

void RobotParameters::setWalkParameter(int p, float s)
{
    
    if(p>ParametersSize || p < 0)
    {
        std::cerr << "Error Invalid parameter" << std::endl;
        return ;
    }
    WalkParameters[p]=s;
}


int RobotParameters::writeWalkParametersFromFile(const char * filename)
{
    fprintf(stderr,"Saving configuration to `%s` \n",filename);
    FILE * fp = fopen(filename,"w");
    if (fp!=0)
    {
        fprintf(fp,"%s\n",robotName);
        fprintf(fp,"%u\n",ParametersSize);
        unsigned int i=0;
        for (i=0; i<ParametersSize; i++)
        {
            fprintf(fp,"%f\n",WalkParameters[i]);
        }
        fclose(fp);
        return 1;
    }
    
    return 0;
}


int RobotParameters::readWalkParametersFromFile(const char * filename)
{
    fprintf(stderr,"Reading configuration from `%s` \n",filename);
    FILE * fp = fopen(filename,"r");
    if (fp!=0)
    {
        fscanf(fp, "%s\n", robotName );
        unsigned int numberOfArgumentsInSavedFile=0;
        fscanf(fp, "%u\n", &numberOfArgumentsInSavedFile);
        if (numberOfArgumentsInSavedFile!=ParametersSize)
        {
            fprintf(stderr,"File format is changed , will not load it because it might corrupt state , instead using default values..\n");
            fclose(fp);
            return 0;
        }
        else
        {
            unsigned int i=0;
            for (i=0; i<ParametersSize; i++)
            {
                fscanf(fp,"%f\n",&WalkParameters[i]);
            }
        }
        fclose(fp);
        return 1;
    }
    
    return 0;
}




void RobotParameters::printWalkParameters()
{
    fprintf(stderr,"Active Walk Parameters..\n");
    fprintf(stderr,"=================================\n");
    unsigned int i=0;
    fprintf(stderr,"Robot Name : %s \n",robotName);
    for (i=0; i<ParametersSize; i++)
    {
        fprintf(stderr,"Param %s : %f\n", robotParameterNames[i] ,WalkParameters[i] );
    }
    fprintf(stderr,"=================================\n");
}
