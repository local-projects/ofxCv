#include "ofxCv/Kalman.h"

namespace ofxCv {
	
	// based on code from:
	// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
	
	using namespace cv;
	
	template <class T>
	void KalmanPosition_<T>::init(T smoothness, T rapidness, bool bUseAccel, bool bUseJerk) {
		// Re-initialize the flag for updating
		bFirstUpdate = true;
		// Re-init the params
		if (bUseJerk && bUseAccel) {
			KF.init(12, 3, 0); // 12 variables (position+velocity+accel) and 3 measurements (position)

			KF.transitionMatrix = (Mat_<T>(12, 12) <<
				  1,   0,   0,   1,   0,   0, 0.5,   0,   0,0.25,   0,   0,
				  0,   1,   0,   0,   1,   0,   0, 0.5,   0,   0,0.25,   0,
				  0,   0,   1,   0,   0,   1,   0,   0, 0.5,   0,   0,0.25,
				  0,   0,   0,   1,   0,   0,   1,   0,   0, 0.5,   0,   0,
				  0,   0,   0,   0,   1,   0,   0,   1,   0,   0, 0.5,   0,
				  0,   0,   0,   0,   0,   1,   0,   0,   1,   0,   0, 0.5,
				  0,   0,   0,   0,   0,   0,   1,   0,   0,   1,   0,   0,
				  0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   1,   0,
				  0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   1,
				  0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,   0,
				  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   0,
				  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1);

			measurement = Mat_<T>::zeros(3, 1);

			KF.statePre = Mat_<T>::zeros(12, 1);
		}
		else if( bUseAccel ) {
			KF.init(9, 3, 0); // 9 variables (position+velocity+accel) and 3 measurements (position)
			
			KF.transitionMatrix = (Mat_<T>(9, 9) <<
									1,0,0,1,0,0,0.5,0,0,
									0,1,0,0,1,0,0,0.5,0,
									0,0,1,0,0,1,0,0,0.5,
									0,0,0,1,0,0,1,0,0,
									0,0,0,0,1,0,0,1,0,
									0,0,0,0,0,1,0,0,1,
									0,0,0,0,0,0,1,0,0,
									0,0,0,0,0,0,0,1,0,
									0,0,0,0,0,0,0,0,1);
			
			measurement = Mat_<T>::zeros(3, 1);
			
			KF.statePre = Mat_<T>::zeros(9, 1);
		} else {
			KF.init(6, 3, 0); // 6 variables (position+velocity) and 3 measurements (position)
			
			KF.transitionMatrix = (Mat_<T>(6, 6) <<
									1,0,0,1,0,0,
									0,1,0,0,1,0,
									0,0,1,0,0,1,
									0,0,0,1,0,0,
									0,0,0,0,1,0,
									0,0,0,0,0,1);
			
			measurement = Mat_<T>::zeros(3, 1);
			
			KF.statePre = Mat_<T>::zeros(6, 1);
		}
		setIdentity(KF.measurementMatrix);
		setIdentity(KF.processNoiseCov, Scalar::all(smoothness));
		setIdentity(KF.measurementNoiseCov, Scalar::all(rapidness));
		setIdentity(KF.errorCovPost, Scalar::all(.1));
	}
	
	template <class T>
	void KalmanPosition_<T>::update(const glm::vec3& p) {

		// If this is the first update, then set the initial state to be the current value
		if (bFirstUpdate) {
			bFirstUpdate = false;

			KF.statePre.at<T>(0, 0) = KF.statePost.at<T>(0, 0) = p[0];
			KF.statePre.at<T>(1, 0) = KF.statePost.at<T>(1, 0) = p[1];
			KF.statePre.at<T>(2, 0) = KF.statePost.at<T>(2, 0) = p[2];
		}

		// First predict, to update the internal statePre variable
		prediction = KF.predict();

		// The "correct" phase that is going to use the predicted value and our measurement
		measurement(0) = p.x;
		measurement(1) = p.y;
		measurement(2) = p.z;
		estimated = KF.correct(measurement);
	}
	
	template <class T>
	glm::vec3 KalmanPosition_<T>::getPrediction()
	{
		return glm::vec3(prediction(0), prediction(1), prediction(2));
	}

	template <class T>
	glm::mat3x3 KalmanPosition_<T>::getPredictionAll()
	{
		return glm::mat3x3(
			prediction(0), prediction(1), prediction(2),
			prediction(3), prediction(4), prediction(5),
			prediction(6), prediction(7), prediction(8));
	}
	
	template <class T>
	glm::vec3 KalmanPosition_<T>::getEstimation()
	{
		return glm::vec3(estimated(0), estimated(1), estimated(2));
	}
	
	template <class T>
	glm::vec3 KalmanPosition_<T>::getVelocity()
	{
		return glm::vec3(estimated(3), estimated(4), estimated(5));
	}
	
	template class KalmanPosition_<float>;
	
	template <class T>
	void KalmanEuler_<T>::init(T smoothness, T rapidness, bool bUseAccel, bool bUseJerk) {
		KalmanPosition_<T>::init(smoothness, rapidness, bUseAccel, bUseJerk);
		eulerPrev.x = 0.f;
		eulerPrev.y = 0.f;
		eulerPrev.z = 0.f;
	}
	
	template <class T>
	void KalmanEuler_<T>::update(const ofQuaternion& q) {
		// warp to appropriate dimension
		glm::vec3 euler = q.getEuler();
		for( int i = 0; i < 3; i++ ) {
			float rev = floorf((eulerPrev[i] + 180) / 360.f) * 360;
			euler[i] += rev;
			if( euler[i] < -90 + rev && eulerPrev[i] > 90 + rev ) euler[i] += 360;
			else if( euler[i] > 90 + rev && eulerPrev[i] < -90 + rev ) euler[i] -= 360;
		}
        
		// If this is the first update, set the initial values
		if (KalmanPosition::bFirstUpdate) {
			KalmanPosition::bFirstUpdate = false;

            KalmanPosition_<T>::KF.statePre.template at<T>(0, 0) = KalmanPosition_<T>::KF.statePost.template at<T>(0, 0) = euler[0];
            KalmanPosition_<T>::KF.statePre.template at<T>(1, 0) = KalmanPosition_<T>::KF.statePost.template at<T>(1, 0) = euler[1];
            KalmanPosition_<T>::KF.statePre.template at<T>(2, 0) = KalmanPosition_<T>::KF.statePost.template at<T>(2, 0) = euler[2];
		}

		KalmanPosition_<T>::update(euler);
		eulerPrev = euler;
	}
	
	template <class T>
	ofQuaternion KalmanEuler_<T>::getPrediction()
	{
		ofQuaternion q;
		q.set(0, 0, 0, 1);
		glm::vec3 euler = KalmanPosition_<T>::getPrediction();
		
		q.makeRotate(euler.x, glm::vec3(1, 0, 0), euler.z, glm::vec3(0, 0, 1), euler.y, glm::vec3(0, 1, 0));
		
		return q;
	}
	
	template <class T>
	ofQuaternion KalmanEuler_<T>::getEstimation()
	{
		ofQuaternion q;
		q.set(0, 0, 0, 1);
		glm::vec3 euler = KalmanPosition_<T>::getEstimation();
		
		q.makeRotate(euler.x, glm::vec3(1, 0, 0), euler.z, glm::vec3(0, 0, 1), euler.y, glm::vec3(0, 1, 0));
		
		return q;
	}
	
	template class KalmanEuler_<float>;
	
}
