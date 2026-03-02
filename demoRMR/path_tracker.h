#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

// lets go bitches

class PathTracker
{
    double _setpointX = 0;
    double _setpointY = 0;

public:
    void setSetpoint(double x, double y);
    double getSetpointX() { return _setpointX; }
    double getSetpointY() { return _setpointY; }
};

#endif // PATH_TRACKER_H
