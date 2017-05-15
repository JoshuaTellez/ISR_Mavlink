#include "soci/soci.h"
#include "soci/soci-mysql.h"
#include <iostream>
#include <istream>
#include <ostream>
#include <fstream>
#include <string>
#include <exception>


using namespace soci;
using namespace std;

/*THINGS TO ADD
 * 	--DONE-- Add a value to be marked if data is retrieved.
 * 	Create a function to dump an x amount of latest data
 * 		-Having problems with 2d arrays with undefined ranges;
 * 	--DONE-- Add function to push special sql queries
 * 	--DONE-- Add functions to grab specific parts or all of each table. Format it and return it correctly.
 * 	Create a readme with all available functions and descriptions of arguments and use.
 * 	 How to grab multi lines: sql << select id from gps where id >=((select max(id) from gps) - 4) && pulled = 0; for controlled pull
 */


session sql;
void UAV_DatabaseConnect(string dbname,string username,string password)
{
    try
    {
    	string sqlconinfo = "db=" + dbname + " " + "user=" + username + " " + "password=" + password;
    	cout << sqlconinfo << endl; //For Debug
    	sql.open(mysql, sqlconinfo);
    	sql << "CREATE TABLE IF NOT EXISTS gps_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS gps_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS heading_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS heading_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS waypoint_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS waypoint_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS target_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS target_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL)";
    	sql << "CREATE TABLE IF NOT EXISTS pixeltarget (id int NOT NULL PRIMARY KEY, x INT NOT NULL, y INT NOT NULL)";
    	sql << "SET GLOBAL max_allowed_packet=1073741824;";
    	sql << "SET GLOBAL net_read_timeout = 200 ;";
    	sql << "SET GLOBAL connect_timeout = 200 ;";
    	sql << "SET GLOBAL max_connections = 2000 ;";
    	//sql.set_log_stream(ostream);
    }
    catch (exception const &e)
    {
        cerr << "Error: " << e.what() << '\n';
    }
}

void UAV_InsertGPS_LOCAL(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS gps_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlgpsinfo = (sql.prepare << "INSERT INTO gps_local (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude),use(latitude),use(longitude),use(0));
	sqlgpsinfo.execute(true);
}
void UAV_InsertGPS_ALT(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS gps_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlgpsinfo = (sql.prepare << "INSERT INTO gps_alt (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude),use(latitude),use(longitude),use(0));
	sqlgpsinfo.execute(true);
}
void UAV_InsertHEADING_LOCAL(double yaw, double pitch, double roll)
{
	sql << "CREATE TABLE IF NOT EXISTS heading_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlheadinginfo = (sql.prepare << "INSERT INTO heading_local (yaw,pitch,roll,pulled) VALUES (:f,:f,:f,:b)", use(yaw), use(pitch), use(roll),use(0));
	sqlheadinginfo.execute(true);
}
void UAV_InsertHEADING_ALT(double yaw, double pitch, double roll)
{
	sql << "CREATE TABLE IF NOT EXISTS heading_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, yaw double NOT NULL, pitch double NOT NULL, roll double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlheadinginfo = (sql.prepare << "INSERT INTO heading_alt (yaw,pitch,roll,pulled) VALUES (:f,:f,:f,:b)", use(yaw), use(pitch), use(roll),use(0));
	sqlheadinginfo.execute(true);
}
void UAV_InsertWAYPOINT_LOCAL(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS waypoint_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlwaypointinfo = (sql.prepare << "INSERT INTO waypoint_local (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude), use(latitude), use(longitude),use(0));
	sqlwaypointinfo.execute(true);
}
void UAV_InsertWAYPOINT_ALT(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS waypoint_alt (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL, time TIMESTAMP DEFAULT CURRENT_TIMESTAMP, pulled BOOL NOT NULL)";
	statement sqlwaypointinfo = (sql.prepare << "INSERT INTO waypoint_alt (altitude,latitude,longitude,pulled) VALUES (:f,:f,:f,:b)", use(altitude), use(latitude), use(longitude),use(0));
	sqlwaypointinfo.execute(true);
}
void UAV_InsertTARGET_LOCAL(double altitude, double latitude, double longitude)
{
	sql << "CREATE TABLE IF NOT EXISTS target_local (id int NOT NULL AUTO_INCREMENT PRIMARY KEY, altitude double NOT NULL, latitude double NOT NULL, longitude double NOT NULL)";
	statement sqltargetinfo = (sql.prepare << "REPLACE INTO target_local (id,altitude,latitude,longitude) VALUES (1,:f,:f,:f)", use(altitude), use(latitude), use(longitude));
	sqltargetinfo.execute(true);
}
void UAV_PullGPS_LOCAL(double a[3],int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM gps_local WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE gps_local SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullGPS_ALT(double a[3],int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM gps_alt WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE gps_alt SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullHEADING_LOCAL(double a[3], int pulled)
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	int tempid = 0;
	sql << "SELECT id,yaw,pitch,roll FROM heading_local WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempyaw),into(temppit),into(temprol);
	if(pulled==0)
		sql << "UPDATE heading_local SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_PullHEADING_ALT(double a[3], int pulled)
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	int tempid = 0;
	sql << "SELECT id,yaw,pitch,roll FROM heading_alt WHERE pulled = 0 LIMIT 1;", into(tempid), into(tempyaw),into(temppit),into(temprol);
	if(pulled==0)
		sql << "UPDATE heading_alt SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_PullWAYPOINT_LOCAL(double a[3], int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM waypoint_local WHERE pulled = 0 LIMIT 1", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE waypoint_local SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullWAYPOINT_ALT(double a[3], int pulled)
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	int tempid = 0;
	sql << "SELECT id,altitude,latitude,longitude FROM waypoint_alt WHERE pulled = 0 LIMIT 1", into(tempid), into(tempalt),into(templat),into(templon);
	if(pulled==0)
		sql << "UPDATE waypoint_alt SET pulled = 1 WHERE id = :row",use(tempid);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullTARGET_LOCAL(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM target_local where id = 1;", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullLatestGPS_LOCAL(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM gps_local WHERE id = (select max(id) from gps_local);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullLatestGPS_ALT(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM gps_alt WHERE id = (select max(id) from gps_alt);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullLatestHEADING_LOCAL(double a[3])
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	sql << "SELECT altitude,latitude,longitude FROM heading_local WHERE id = (select max(id) from heading_local);", into(tempyaw),into(temppit),into(temprol);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_PullLatestHEADING_ALT(double a[3])
{
	double tempyaw = 0;
	double temppit = 0;
	double temprol = 0;
	sql << "SELECT altitude,latitude,longitude FROM heading_alt WHERE id = (select max(id) from heading_alt);", into(tempyaw),into(temppit),into(temprol);
	a[0] = tempyaw;
	a[1] = temppit;
	a[2] = temprol;
}
void UAV_PullLatestWAYPOINT_LOCAL(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM waypoint_local WHERE id = (select max(id) from waypoint_local);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_PullLatestWAYPOINT_ALT(double a[3])
{
	double tempalt = 0;
	double templat = 0;
	double templon = 0;
	sql << "SELECT altitude,latitude,longitude FROM waypoint_alt WHERE id = (select max(id) from waypoint_alt);", into(tempalt),into(templat),into(templon);
	a[0] = tempalt;
	a[1] = templat;
	a[2] = templon;
}
void UAV_DropGPS_LOCAL()
{
	sql << "DROP TABLE IF EXISTS gps_local";
}
void UAV_DropGPS_ALT()
{
	sql << "DROP TABLE IF EXISTS gps_alt";
}
void UAV_DropHEADING_LOCAL()
{
	sql << "DROP TABLE IF EXISTS heading_local";
}
void UAV_DropHEADING_ALT()
{
	sql << "DROP TABLE IF EXISTS heading_alt";
}
void UAV_DropWAYPOINT_LOCAL()
{
	sql << "DROP TABLE IF EXISTS waypoint_local";
}
void UAV_DropWAYPOINT_ALT()
{
	sql << "DROP TABLE IF EXISTS waypoint_alt";
}
void UAV_DropTARGET_LOCAL()
{
	sql << "DROP TABLE IF EXISTS waypoint_local";
}
void UAV_DropTARGET_ALT()
{
	sql << "DROP TABLE IF EXISTS waypoint_alt";
}
void UAV_Query(string query)
{
	sql << query;
}
void UAV_DatabaseDisconnect()
{
	sql.close();
}
void UAV_InsertPixelTARGET(int x, int y)
{
	sql << "CREATE TABLE IF NOT EXISTS pixeltarget (id int NOT NULL PRIMARY KEY, x INT NOT NULL, y INT NOT NULL)";
	statement sqlpixeltargetinfo = (sql.prepare << "REPLACE INTO pixeltarget (id,x,y) VALUES (1,:i,:i)", use(x), use(y));
	sqlpixeltargetinfo.execute(true);
}
void UAV_PullPixelTARGET(int xy[2])
{
	sql << "SELECT x,y FROM pixeltarget where id = 1;", into(xy[0]),into(xy[1]);
}


