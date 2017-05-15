
#ifndef UAV_Database_H
#define UAV_Database_H

#include <string>

void UAV_DatabaseConnect(std::string,std::string,std::string);
void UAV_InsertGPS_LOCAL(double,double,double);
void UAV_InsertHEADING_LOCAL(double,double,double);
void UAV_InsertWAYPOINT_LOCAL(double,double,double);
void UAV_InsertTARGET_LOCAL(double,double,double);
void UAV_InsertGPS_ALT(double,double,double);
void UAV_InsertHEADING_ALT(double,double,double);
void UAV_InsertWAYPOINT_ALT(double,double,double);
void UAV_PullGPS_LOCAL(double *a,int);
void UAV_PullHEADING_LOCAL(double *a,int);
void UAV_PullWAYPOINT_LOCAL(double *a,int);
void UAV_PullTARGET_LOCAL(double *a);
void UAV_PullGPS_ALT(double *a,int);
void UAV_PullHEADING_ALT(double *a,int);
void UAV_PullWAYPOINT_ALT(double *a,int);
void UAV_PullLatestGPS_LOCAL(double *a);
void UAV_PullLatestWAYPOINT_LOCAL(double *a);
void UAV_PullLatestHEADING_LOCAL(double *a);
void UAV_PullLatestGPS_ALT(double *a);
void UAV_PullLatestWAYPOINT_ALT(double *a);
void UAV_PullLatestHEADING_ALT(double *a);
void UAV_DropGPS_LOCAL();
void UAV_DropHEADING_LOCAL();
void UAV_DropWAYPOINT_LOCAL();
void UAV_DropTARGET_LOCAL();
void UAV_DropGPS_ALT();
void UAV_DropHEADING_ALT();
void UAV_DropWAYPOINT_ALT();
void UAV_DropTARGET_ALT();
void UAV_Query(std::string);
void UAV_DatabaseDisconnect();
void UAV_InsertPixelTARGET(int,int);
void UAV_PullPixelTARGET(int *xy);
#endif
