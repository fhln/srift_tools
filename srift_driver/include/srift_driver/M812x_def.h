#ifndef M812X_DEF_H
#define M812X_DEF_H

#include <Eigen/Dense>
using namespace Eigen;

#define M812X_CHN_NUMBER	6
MatrixXd m_dResultChValue=MatrixXd::Zero(1,M812X_CHN_NUMBER);   //engineering output of each channel
MatrixXd m_dDecouplingValue=MatrixXd::Zero(1,M812X_CHN_NUMBER); //final output
MatrixXd m_nADCounts=MatrixXd::Zero(1,M812X_CHN_NUMBER);        //ad output
MatrixXd m_dAmpZero=MatrixXd::Zero(1,M812X_CHN_NUMBER);         //read
MatrixXd m_dChnGain=MatrixXd::Zero(1,M812X_CHN_NUMBER);         //read
MatrixXd m_dChnEx=MatrixXd::Zero(1,M812X_CHN_NUMBER);           //read
MatrixXd m_dSens=MatrixXd::Zero(1,M812X_CHN_NUMBER);
MatrixXd m_dExc=MatrixXd::Zero(1,M812X_CHN_NUMBER);

MatrixXd m_dDecouplingCoefficient(6, 6);

#endif
