#ifndef CALC_ENTROPY_H
#define CALC_ENTROPY_H
#include <vector>


namespace CalcEntropy{
  struct ConfigDist {
    double dist;
    int id;
  };

  double calcDifferentialEntropy(std::vector<double> dist);
  double calcCondDisEntropy(std::vector<ConfigDist> p, double binSize);
  double calcIG(std::vector<ConfigDist> p, double binSize, int numConfigs);



}

#endif

