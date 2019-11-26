/*
Authored By
Mark McCutcheon, Clutch Design Solutions
Gitub: https://github.com/markmccutch
Email: mark.mccutcheon29@gmail.com
*/

#ifndef _SimpleKalman_h_
#define _SimpleKalman_h_


class SimpleKalman{

  public:
    SimpleKalman();

    void setPosition(double pos);
    void setVelocity(double velocity);

    void setEstimatedCovariance(float cov);
    void setProcessError(float err_q);
    void setSensorError(float err_r);
    void setTranslation(float h);

    void predictEstimate(double u, double dt);
    void updateEstimate(double pos, double velocity, double dt);

    double getPosition();
    double getVelocity();

  private:
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
