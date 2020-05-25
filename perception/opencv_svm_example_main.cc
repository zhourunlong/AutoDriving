// Copyright @2020 Pony AI Inc. All rights reserved.
// Modified from https://docs.opencv.org/2.4/doc/tutorials/ml/introduction_to_svm/introduction_to_svm.html

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

using namespace cv;

int main()
{
  // When you train the model:
  // Set up training data
  float labels[4] = {1.0, -1.0, -1.0, -1.0};
  Mat labelsMat(4, 1, CV_32FC1, labels);

  float trainingData[4][2] = { {501, 10}, {255, 10}, {501, 255}, {10, 501} };
  Mat trainingDataMat(4, 2, CV_32FC1, trainingData);

  // Set up SVM's parameters
  CvSVMParams params;
  params.svm_type    = CvSVM::C_SVC;
  params.kernel_type = CvSVM::LINEAR;
  params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);

  // Train the SVM
  CvSVM svm;
  svm.train(trainingDataMat, labelsMat, Mat(), Mat(), params);

  // Save the model
  svm.save("result_model.txt");

  ///////////////////////////////////////////////////////
  // When you use the trained model in your binary:
  // Load the model
  CvSVM svm2;
  svm2.load("result_model.txt");

  // Predict on a feature vector.
  Mat sampleMat = (Mat_<float>(1,2) << 500, 255);
  float response = svm2.predict(sampleMat);

  std::cout << "Prediction result: " << response << std::endl;
  return 0;
}

