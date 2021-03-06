/*
Authored By
Mark McCutcheon, Clutch Design Solutions
Gitub: https://github.com/markmccutch
Email: mark.mccutcheon29@gmail.com
*/

#ifndef _SimpleKalman_h_
#define _SimpleKalman_h_
#define EARTH_RADIUS 6368.061

class SimpleKalman{

  public:
    SimpleKalman();

    float GPStokm(float lon1, float lon2, float lat1, float lat2);
    float mpermillis_to_ms(float m_millis);

    void setPosition(double pos);
    void setVelocity(double velocity);

    void setEstimatedCovariance(float cov);
    void setProcessError(float err_q);
    void setSensorError(float err_r);
    void setTranslation(float h);

    void predictEstimate(double u, double t1);
    void updateEstimate(double pos, double t2);

    double getPosition();
    double getVelocity();

  private:
    double _time1;
    double _time2;

    double _initial_pos;
    double _lastPosition;
    double _lastVelocity;
    double _position;
    double _velocity;
    double _estimatedPosition;
    double _estimatedVelocity;

    float _processErrorQ;
    float _sensorErrorR;
    float _translationH;

    float _P[2][2]; //error covariance matrix;

};

#endif
