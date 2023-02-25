#ifndef TRAILER_PARAMS_H
#define TRAILER_PARAMS_H

class TractorTrailerParam {
 public:
  TractorTrailerParam() = default;
  ~TractorTrailerParam() = default;
  static TractorTrailerParam &GetInstance() {
    static TractorTrailerParam instance;
    return instance;
  }

  double L1 = 1.62;  // Tractor wheel base
  double W1 = 1.56;  // Tractor width
  double k1 = 0.33;  // Tractor rear tyre offset
  double k2 = 0.58;  // Tractor front tyre offset
  double k3 = 0.17;  // Trailer front tyre offset
  double k4 = 0.14;  // Trailer rear tyre offset
  double W2 = 0.9;   // Trailer width
  double M1 = 0.57;  // Tractor hitch offset
  double L2 = 2.04;  // Trailer wheel base
};

#endif