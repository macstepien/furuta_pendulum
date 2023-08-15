#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace furuta_pendulum {

struct FurutaPendulumParameters {
  FurutaPendulumParameters() { computeInertiaTerms(); }

  void display() {
    std::cerr << "Furuta Pendulum parameters: "
              << "\n";
    std::cerr << "m1: " << m1_ << "\n";
    std::cerr << "m2: " << m2_ << "\n";
    std::cerr << "l1: " << l1_ << "\n";
    std::cerr << "l2: " << l2_ << "\n";
    std::cerr << "L1: " << L1_ << "\n";
    std::cerr << "L2: " << L2_ << "\n";
    std::cerr << "b1: " << b1_ << "\n";
    std::cerr << "b2: " << b2_ << "\n";
    std::cerr << "J1: " << J1_ << "\n";
    std::cerr << "J2: " << J2_ << "\n";
    std::cerr << "J2_hat: " << J2_hat_ << "\n";
    std::cerr << "J0_hat: " << J0_hat_ << "\n";
  }

  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << "\n #### Furuta Pendulum Parameters:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, m1_, fieldName + ".m1", verbose);
    loadData::loadPtreeValue(pt, m2_, fieldName + ".m2", verbose);
    loadData::loadPtreeValue(pt, l1_, fieldName + ".l1", verbose);
    loadData::loadPtreeValue(pt, l2_, fieldName + ".l2", verbose);
    loadData::loadPtreeValue(pt, L1_, fieldName + ".L1", verbose);
    loadData::loadPtreeValue(pt, L2_, fieldName + ".L2", verbose);
    loadData::loadPtreeValue(pt, b1_, fieldName + ".b1", verbose);
    loadData::loadPtreeValue(pt, b2_, fieldName + ".b2", verbose);
    loadData::loadPtreeValue(pt, J1_, fieldName + ".J1", verbose);
    loadData::loadPtreeValue(pt, J2_, fieldName + ".J2", verbose);
    computeInertiaTerms();
    if (verbose) {
      std::cerr << " #### =============================================================================\n" << std::endl;
    }
  }

  scalar_t m1_;
  scalar_t m2_;
  scalar_t l1_;
  scalar_t l2_;
  scalar_t L1_;
  scalar_t L2_;
  scalar_t b1_;
  scalar_t b2_;
  scalar_t J1_;
  scalar_t J2_;
  scalar_t J2_hat_;
  scalar_t J0_hat_;

  scalar_t g_ = 9.80665;

 private:
  void computeInertiaTerms() {
    J2_hat_ = J2_ + m2_ * l2_ * l2_;
    J0_hat_ = J1_ + m1_ * l1_ * l1_ + m2_ * L1_ * L1_;
  }
};

}  // namespace furuta_pendulum
}  // namespace ocs2
