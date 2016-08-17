#include <calcEntropy.h>
#include <iostream>
#include <algorithm>
#include <math.h>


struct Bin {
  // std::vector<CalcEntropy::ConfigDist> element;
  std::vector<int> id;
};


/*
 * Given a sorted vector of config dists, returns the histogram of configurations
 *   based on "dist"
 */
static std::vector<Bin> histogram(std::vector<CalcEntropy::ConfigDist> c, double binSize){
  double min = c[0].dist;
  
  int m = c.size()-1;
  while(m>0 && c[m].dist > 999){
    m--;
  }

  double max = c[m].dist;
  

  int nbins = (int)((max-min)/binSize) + 2;
  double binValue = min + binSize;
  int binNum = 0;
  std::vector<Bin> hist;

  hist.resize(nbins);


  // std::cout <<"Max: " << max << " Min: " << min << std::endl;
  // std::cout <<"c size " << c.size() << " hist size: " << hist.size() << std::endl;

  int i=0;
  while(i < c.size()){
    if(c[i].dist < binValue || c[i].dist > 999){
      hist[binNum].id.push_back(c[i].id);
      i++;
    } else {
      binNum++;
      binValue += binSize;
    }
  }
  return hist;
}


/*
 * Given a sorted vector of doubles, returns the histogram 
 * of the data in n evenly spaced bins
 */
static std::vector<double> histogram(std::vector<double> dist, int nbins, double* binSize){
  std::vector<double> hist;
  hist.resize(nbins);
  double min = dist[0];
  double max = dist[dist.size()-1];
  *binSize = (max-min)/nbins;
  double binNum = 0;
  double binValue = min + *binSize;

  if(max == min){
    hist.resize(0);
    return hist;
  }

  int i = 0;
  while(i < dist.size()){
    // std::cout << dist[i] << std::endl;
    if(dist[i] < binValue){
      hist[binNum]++;
      i++;
    } else {
      binNum++;
      binValue += *binSize;
    }
  }
  return hist;
}


static double calcEntropyOfBin(Bin bin){
  double entropy = 0;
  double numPointsInBin = bin.id.size();
  std::sort(bin.id.begin(), bin.id.end());

  int unique_id_count = 0;
  int unique_id = -1;
  for(int i=0; i<bin.id.size(); i++){
    if(unique_id == bin.id[i]){
      unique_id_count++;
    } else {
      if(unique_id_count > 0){
	 double p = (double)unique_id_count / numPointsInBin;
	 entropy -= p * log(p);
      }
      unique_id_count = 1;
      unique_id = bin.id[i];
    }
  }
  if(unique_id_count > 0){
    double p = (double)unique_id_count / numPointsInBin;
    entropy -= p * log(p);
  }

  
  return entropy;
}


/**
 * Ordering function for COnfigDist sort.
 *  
 */
bool distOrdering(const CalcEntropy::ConfigDist &left, const CalcEntropy::ConfigDist &right) {
  return left.dist < right.dist;
}


namespace CalcEntropy{

  /*
   * Calculates entropy of element of dist distribution
   *  Uses a histogram then evalutes the entropy
   *  of the discrete elements
   */
  double calcDifferentialEntropy(std::vector<double> dist){
    // std::cout << "Calculating Entropy" << std::endl;
    // std::cout << "size is " << dist.size() << std::endl;
    std::sort(dist.begin(), dist.end());
    double binSize;
    std::vector<double> hist = histogram(dist, dist.size()/5, &binSize);
    if(hist.size() == 0){
      return 0;
    }
    
    double entropy = 0;
    for(int i=0; i < hist.size(); i++){
      double f = hist[i] / dist.size();
      if(f > 0){
	entropy += -f * log(f/binSize);
      }
      // std::cout << hist[i] << std::endl;
    }

    return entropy;
  }

  /*
   *  Calculates conditional discrete entropy of histogram of distance
   */
  double calcCondDisEntropy(std::vector<ConfigDist> p, double binSize)
  {
    std::sort(p.begin(), p.end(), &distOrdering);
    std::vector<Bin> hist = histogram(p, binSize);

    double totalPoints = p.size();
    double entropy = 0;


    for(int binId = 0; binId < hist.size(); binId++){
      // std::cout << "Bin " << binId << std::endl;
      for(int j=0; j<hist[binId].id.size(); j++){
	// std::cout << hist[binId].id[j] << ", ";
      }

      Bin bin = hist[binId];

      double p_bin = bin.id.size()/totalPoints;
      entropy += p_bin * calcEntropyOfBin(bin);
      // std::cout << std::endl; 
      // std::cout << "Entropy: " << calcEntropyOfBin(bin);
      // std::cout << " Probability: " << p_bin << std::endl;
    }

    // std::cout << "Entropy: " << entropy << std::endl;
    return entropy;
    
  }
  
  double calcIG(std::vector<ConfigDist> distances, double binSize, int numConfigs)
  {
    double H_Y_given_X = calcCondDisEntropy(distances, binSize);
    double p = 1.0 / (double)numConfigs;
    double H_Y = -log(p);

    // std::cout << "IG: " <<  H_Y - H_Y_given_X << std::endl;
    return H_Y - H_Y_given_X;
  }

}
