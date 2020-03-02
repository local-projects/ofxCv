#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofVectorMath.h"

namespace ofxCv {
	
	// Kalman filter for positioning
	template <class T>
	class KalmanPosition_ {
	protected:
		cv::KalmanFilter KF;
		cv::Mat_<T> measurement, prediction, estimated;
		bool bFirstUpdate = true;
	public:
		// smoothness, rapidness: smaller is more smooth/rapid
		// bUseAccel: set true to smooth out velocity
		void init(T smoothness = 0.1, T rapidness = 0.1, bool bUseAccel = false, bool bUseJerk = false);
		void update(const glm::vec3&);
		glm::vec3 getPrediction();
		glm::mat3x3 getPredictionAll();
		glm::vec3 getEstimation();
		glm::vec3 getVelocity();
		void predict() { prediction = KF.predict(); }; // progress forward one time step
	};
	
	typedef KalmanPosition_<float> KalmanPosition;
	
	// Kalman filter for orientation
	template <class T>
	class KalmanEuler_ : public KalmanPosition_<T> {
	protected:
		glm::vec3 eulerPrev; // used for finding appropriate dimension
	public:
		void init(T smoothness = 0.1, T rapidness = 0.1, bool bUseAccel = false, bool bUseJerk = false);
		void update(const ofQuaternion&);
		ofQuaternion getPrediction();
		ofQuaternion getEstimation();
		//ofQuaternion getVelocity();
	};
	
	typedef KalmanEuler_<float> KalmanEuler;	
}
