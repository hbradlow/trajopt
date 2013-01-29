#include "trajopt/problem_description.hpp"
#include "trajopt/common.hpp"
#include <boost/foreach.hpp>
#include "ipi/logging.hpp"
#include "ipi/sco/expr_ops.hpp"
#include "trajopt/kinematic_constraints.hpp"
#include "trajopt/collision_avoidance.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/plot_callback.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/eigen_slicing.hpp"
#include <boost/algorithm/string.hpp>
#include <limits>
using namespace Json;
using namespace std;
using namespace OpenRAVE;
using namespace trajopt;
using namespace util;


#define FAIL_IF_FALSE(expr) if (!expr) {\
    throw MY_EXCEPTION( "expected true: " #expr);\
  }

namespace {


bool gRegisteredMakers = false;
void RegisterMakers() {

  CostInfo::RegisterMaker("pose", &PoseCostInfo::create);
  CostInfo::RegisterMaker("joint_vel", &JointVelCostInfo::create);
  CostInfo::RegisterMaker("collision", &CollisionCostInfo::create);
  CostInfo::RegisterMaker("continuous_collision", &ContinuousCollisionCostInfo::create);

  CntInfo::RegisterMaker("joint", &JointConstraintInfo::create);
  CntInfo::RegisterMaker("pose", &PoseCntInfo::create);
  CntInfo::RegisterMaker("rel_pose", &RelPoseCntInfo::create);

  CntInfo::RegisterMaker("pos", &PosCntInfo::create);
  CntInfo::RegisterMaker("cart_vel", &CartVelCntInfo::create);

  gRegisteredMakers = true;
}

RobotAndDOFPtr RADFromName(const string& name, RobotBasePtr robot) {
  if (name == "active") {
    return RobotAndDOFPtr(new RobotAndDOF(robot, robot->GetActiveDOFIndices(), robot->GetAffineDOF(), robot->GetAffineRotationAxis()));
  }
  vector<int> dof_inds;
  int affinedofs = 0;
  Vector rotationaxis(0,0,1);
  vector<string> components;
  boost::split(components, name, boost::is_any_of("+"));
  for (int i=0; i < components.size(); ++i) {
    std::string& component = components[i];
    if (RobotBase::ManipulatorPtr manip = GetManipulatorByName(*robot, component)) {
      vector<int> inds = manip->GetArmIndices();
      dof_inds.insert(dof_inds.end(), inds.begin(), inds.end());
    }
    else if (component == "base") {
      affinedofs |= DOF_X | DOF_Y | DOF_RotationAxis;
    }
    else if (KinBody::JointPtr joint = robot->GetJoint(component)) {
      dof_inds.push_back(joint->GetDOFIndex());
    }
    else throw MY_EXCEPTION( (boost::format("error in reading manip description: %s must be a manipulator, link, or 'base'")%component).str() );
  }
  return RobotAndDOFPtr(new RobotAndDOF(robot, dof_inds, affinedofs, rotationaxis));
}

BoolVec toMask(const VectorXd& x) {
  BoolVec out(x.size());
  for (int i=0; i < x.size(); ++i) out[i] = (x[i] > 0);
  return out;
}

bool allClose(const VectorXd& a, const VectorXd& b) {
  return (a-b).array().abs().maxCoeff() < 1e-4;
}

}

namespace Json { //funny thing with two-phase lookup

void fromJson(const Json::Value& v, Vector3d& x) {
  vector<double> vx;
  fromJsonArray(v, vx, 3);
  x = Vector3d(vx[0], vx[1], vx[2]);
}
void fromJson(const Json::Value& v, Vector4d& x) {
  vector<double> vx;
  fromJsonArray(v, vx, 4);
    x = Vector4d(vx[0], vx[1], vx[2], vx[3]);
}

}

namespace trajopt {

ProblemConstructionInfo* gPCI;

void BasicInfo::fromJson(const Json::Value& v) {
  childFromJson(v, start_fixed, "start_fixed", true);
  childFromJson(v, n_steps, "n_steps");
  childFromJson(v, manip, "manip");
  childFromJson(v, robot, "robot", string(""));

	const Value& vdofs_fixed = v["dofs_fixed"];
	for (int i=0; i < vdofs_fixed.size(); ++i) {
		DblVec row;
		fromJsonArray(vdofs_fixed[i], row);
  	if (row.size() == 1) {
	  	row.push_back(0);
			row.push_back(n_steps);
	  } else if (row.size() == 3) {
	  	if (row[1]<0 || row[1]>n_steps || row[2]<0 || row[2]>n_steps)
				throw MY_EXCEPTION("step0 and step1 should be between 0 and n_steps (inclusive)");
			if (row[2] <= row[1])
				throw MY_EXCEPTION("step1 should be strictly greater than step0");
	  } else {
	  	throw MY_EXCEPTION("number of cols for dofs_fixed should be 1 or 3");
	  }
		dofs_fixed.push_back(row);
	}
}


////
void fromJson(const Json::Value& v, CostInfoPtr& cost) {
  string type;
  childFromJson(v, type, "type");
  cost = CostInfo::fromName(type);
  if (!cost) throw MY_EXCEPTION( (boost::format("failed to construct cost named %s")%type).str() );
  cost->fromJson(v);
  childFromJson(v, cost->name, "name", type);
}
CostInfoPtr CostInfo::fromName(const string& type) {
  if (!gRegisteredMakers) RegisterMakers();
  if (name2maker.find(type) != name2maker.end()) {
    return (*name2maker[type])();
  }
  else {
    RAVELOG_ERROR("There is no cost of type%s\n", type.c_str());
    return CostInfoPtr();
  }
}
map<string, CostInfo::MakerFunc> CostInfo::name2maker;

void CostInfo::RegisterMaker(const std::string& type, MakerFunc f) {
  name2maker[type] = f;
}


////
//// almost copied


void fromJson(const Json::Value& v, CntInfoPtr& cnt) {
  string type;
  childFromJson(v, type, "type");
  IPI_LOG_DEBUG("reading constraint: %s", type);
  cnt = CntInfo::fromName(type);
  if (!cnt) throw MY_EXCEPTION( (boost::format("failed to construct constraint named %s")%type).str() );
  cnt->fromJson(v);
  childFromJson(v, cnt->name, "name", type);
}
CntInfoPtr CntInfo::fromName(const string& type) {
  if (!gRegisteredMakers) RegisterMakers();
  if (name2maker.find(type) != name2maker.end()) {
    return (*name2maker[type])();
  }
  else {
    RAVELOG_ERROR("There is no constraint of type %s\n", type.c_str());
    return CntInfoPtr();
  }
}
map<string,CntInfo::MakerFunc> CntInfo::name2maker;

void InitInfo::fromJson(const Json::Value& v) {
  string type_str;
  childFromJson(v, type_str, "type");
  int n_steps = gPCI->basic_info.n_steps;

  if (type_str == "stationary") {
    data = toVectorXd(gPCI->rad->GetDOFValues()).transpose().replicate(n_steps, 1);;
  }
  else if (type_str == "given_traj") {
    FAIL_IF_FALSE(v.isMember("data"));
    const Value& vdata = v["data"];
    if (vdata.size() != n_steps) {
      throw MY_EXCEPTION("given initialization traj has wrong length");
    }
    int n_dof = gPCI->rad->GetDOF();
    data.resize(n_steps, n_dof);
    for (int i=0; i < n_steps; ++i) {
      DblVec row;
      fromJsonArray(vdata[i], row, n_dof);
      data.row(i) = toVectorXd(row);
    }
  }

}

void ProblemConstructionInfo::fromJson(const Value& v) {
  childFromJson(v, basic_info, "basic_info");

  RobotBasePtr robot = (basic_info.robot=="") ? GetRobot(*env) : GetRobotByName(*env, basic_info.robot);
  if (!robot) {
    throw MY_EXCEPTION("couldn't get robot");
  }
  rad = RADFromName(basic_info.manip, robot);
  if (!rad) {
    throw MY_EXCEPTION( (boost::format("couldn't get manip %s")%basic_info.manip).str() );
  }

  gPCI = this;
  if (v.isMember("costs")) fromJsonArray(v["costs"], cost_infos);
  if (v.isMember("constraints")) fromJsonArray(v["constraints"], cnt_infos);

  childFromJson(v, init_info, "init_info");
  gPCI = NULL;

}
void CntInfo::RegisterMaker(const std::string& type, MakerFunc f) {
  name2maker[type] = f;
}

TrajOptResult::TrajOptResult(OptResults& opt, TrajOptProb& prob) :
  cost_vals(opt.cost_vals),
  cnt_viols(opt.cnt_viols) {
  BOOST_FOREACH(const CostPtr& cost, prob.getCosts()) {
    cost_names.push_back(cost->name());
  }
  BOOST_FOREACH(const ConstraintPtr& cnt, prob.getConstraints()) {
    cnt_names.push_back(cnt->name());
  }
  traj = getTraj(opt.x, prob.GetVars());
}

TrajOptResultPtr OptimizeProblem(TrajOptProbPtr prob, bool plot) {
  BasicTrustRegionSQP opt(prob);
  opt.max_iter_ = 30;
  opt.min_approx_improve_frac_ = .001;
  opt.merit_error_coeff_ = 10;
  if (plot) opt.addCallback(PlotCallback(*prob));
  //  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),boost::ref(*prob->GetRAD()), boost::ref(prob->GetVars()), _1));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();
  return TrajOptResultPtr(new TrajOptResult(opt.results(), *prob));
}

TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo& pci) {
  TrajOptProbPtr prob(new TrajOptProb());

  const BasicInfo& bi = pci.basic_info;
  int n_steps = bi.n_steps;

  prob->m_rad = pci.rad;
  int n_dof = prob->m_rad->GetDOF();


  DblVec lower, upper;
  prob->m_rad->GetDOFLimits(lower, upper);
//  if (n_dof == 3) {
//  	lower = DblVec(3,-std::numeric_limits<double>::infinity());
//  	upper = DblVec(3,std::numeric_limits<double>::infinity());
//  }
//  IPI_LOG_INFO("sizes %d %d\n", lower.size(), upper.size());
//  int s = lower.size();
//  IPI_LOG_INFO("lower limits %d %d %d\n", lower[s-3], lower[s-2], lower[s-1]);
//  IPI_LOG_INFO("upper limits %d %d %d\n", upper[s-3], upper[s-2], upper[s-1]);
  vector<double> vlower, vupper;
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    vlower.insert(vlower.end(), lower.data(), lower.data()+lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data()+upper.size());
    for (unsigned j=0; j < n_dof; ++j) {
      names.push_back( (boost::format("j_%i_%i")%i%j).str() );
    }
  }
//  IPI_LOG_INFO("sizes %d %d %d\n", names.size(), vlower.size(), vupper.size());
//  IPI_LOG_INFO("ndof %d\n", n_dof);
  prob->createVariables(names, vlower, vupper);
  prob->m_traj_vars = VarArray(n_steps, n_dof, prob->vars_.data());

  DblVec cur_dofvals = prob->m_rad->GetDOFValues();

  if (bi.start_fixed) {
  	DblVec clipped_cur_dofvals = prob->m_rad->GetDOFValues();
  	for (int j=0; j < n_dof; ++j) {
      if (cur_dofvals[j]<lower[j]) {
      	IPI_LOG_WARNING("The joint value for the %dth dof is less than the lower limit by %f. The value is clipped.",j,lower[j]-clipped_cur_dofvals[j]);
      	clipped_cur_dofvals[j] = lower[j];
      }
      if (cur_dofvals[j]>upper[j]) {
				IPI_LOG_WARNING("The joint value for the %dth dof is less than the upper limit by %f. The value is clipped.",j,clipped_cur_dofvals[j]-upper[j]);
				clipped_cur_dofvals[j] = upper[j];
			}
    	prob->addLinearConstr(exprSub(AffExpr(prob->m_traj_vars(0,j)), clipped_cur_dofvals[j]), EQ);
    }
		if (pci.init_info.data.rows() > 0 && !(allClose(toVectorXd(cur_dofvals), pci.init_info.data.row(0)) || allClose(toVectorXd(clipped_cur_dofvals), pci.init_info.data.row(0)))) {
			throw MY_EXCEPTION( "robot dof values don't match initialization. I don't know what you want me to use for the dof values");
		}
  }

  for (int i=0; i<bi.dofs_fixed.size(); i++) {
  	int step0 = bi.dofs_fixed[i][1];
  	int dof_ind = ((((int)bi.dofs_fixed[i][0]) % n_dof) + n_dof) % n_dof; // wrap around by n_dof
  	int step1 = bi.dofs_fixed[i][2];
    for (int step=step0+1; step<step1; step++) {
    	IPI_LOG_INFO("dof_ind %d steps[%d:%d]", dof_ind, step0, step);
    	prob->addLinearConstr(exprSub(AffExpr(prob->m_traj_vars(step0,dof_ind)), AffExpr(prob->m_traj_vars(step,dof_ind))), EQ);
    }
  }

  vector<string> cost_names;
  BOOST_FOREACH(const CostInfoPtr& ci, pci.cost_infos) {
    int n_costs_before = prob->getNumCosts();
    ci->hatch(*prob);
    int n_costs_after = prob->getNumCosts();
    for (int i=0; i < n_costs_after - n_costs_before; ++i) cost_names.push_back(ci->name);
  }

  vector<string> cnt_names;
  BOOST_FOREACH(const CntInfoPtr& ci, pci.cnt_infos) {
    int n_cnts_before = prob->getNumConstraints();
    ci->hatch(*prob);
    int n_cnts_after = prob->getNumConstraints();
    for (int i=0; i < n_cnts_after - n_cnts_before; ++i) cnt_names.push_back(ci->name);
  }

  int iCost=0, iCnt=0;
  BOOST_FOREACH(const CostPtr& cost, prob->getCosts()) cost->setName(cost_names[iCost++]);
  BOOST_FOREACH(const ConstraintPtr& cnt, prob->getConstraints()) cnt->setName(cnt_names[iCnt++]);


  prob->SetInitTraj(pci.init_info.data);

  return prob;

}
TrajOptProbPtr ConstructProblem(const Json::Value& root, OpenRAVE::EnvironmentBasePtr env) {
  ProblemConstructionInfo pci(env);
  pci.fromJson(root);
  return ConstructProblem(pci);
}


TrajOptProb::TrajOptProb(int n_steps, RobotAndDOFPtr rad) : m_rad(rad) {
  DblVec lower, upper;
  m_rad->GetDOFLimits(lower, upper);
  int n_dof = m_rad->GetDOF();
  vector<double> vlower, vupper;
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    vlower.insert(vlower.end(), lower.data(), lower.data()+lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data()+upper.size());
    for (unsigned j=0; j < n_dof; ++j) {
      names.push_back( (boost::format("j_%i_%i")%i%j).str() );
    }
  }
  createVariables(names, vlower, vupper);
  m_traj_vars = VarArray(n_steps, n_dof, getVars().data());

}


TrajOptProb::TrajOptProb() {
}


void PoseCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  childFromJson(params, xyz,"xyz");
  childFromJson(params, wxyz,"wxyz");
  childFromJson(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, rot_coeffs,"rot_coeffs", (Vector3d)Vector3d::Ones());

  string linkstr;
  childFromJson(params, linkstr, "link");
  link = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link) {
    throw MY_EXCEPTION( (boost::format("invalid link name: %s")%linkstr).str());
  }
}
CostInfoPtr PoseCostInfo::create() {
  return CostInfoPtr(new PoseCostInfo());
}
void PoseCostInfo::hatch(TrajOptProb& prob) {
  prob.addCost(CostPtr(new CartPoseCost(prob.GetVarRow(timestep), toRaveTransform(wxyz, xyz), rot_coeffs, pos_coeffs, prob.GetRAD(), link)));
}

void PoseCntInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  childFromJson(params, xyz,"xyz");
  childFromJson(params, wxyz,"wxyz");
  childFromJson(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, rot_coeffs,"rot_coeffs", (Vector3d)Vector3d::Ones());

  string linkstr;
  childFromJson(params, linkstr, "link");
  link = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link) {
    throw MY_EXCEPTION( (boost::format("invalid link name: %s")%linkstr).str());
  }
}
CntInfoPtr PoseCntInfo::create() {
  return CntInfoPtr(new PoseCntInfo());
}
void PoseCntInfo::hatch(TrajOptProb& prob) {
  VectorXd coeffs(6); coeffs << rot_coeffs, pos_coeffs;
  prob.addConstr(ConstraintPtr(new CartPoseConstraint(prob.GetVarRow(timestep), toRaveTransform(wxyz, xyz), prob.GetRAD(), link, toMask(coeffs))));
}

void RelPoseCntInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  childFromJson(params, xyz,"xyz");
  childFromJson(params, wxyz,"wxyz");
  childFromJson(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, rot_coeffs,"rot_coeffs", (Vector3d)Vector3d::Ones());

  string linkstr;
  childFromJson(params, linkstr, "link1");
  link1 = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link1) throw MY_EXCEPTION( (boost::format("invalid link1 name: %s")%linkstr).str());
  childFromJson(params, linkstr, "link2");
  link2 = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link2) throw MY_EXCEPTION( (boost::format("invalid link2 name: %s")%linkstr).str());
}
CntInfoPtr RelPoseCntInfo::create() {
  return CntInfoPtr(new RelPoseCntInfo());
}
void RelPoseCntInfo::hatch(TrajOptProb& prob) {
  VectorXd coeffs(6); coeffs << rot_coeffs, pos_coeffs;
  prob.addConstr(ConstraintPtr(new CartRelPoseConstraint(prob.GetVarRow(timestep), toRaveTransform(wxyz, xyz), prob.GetRAD(), link1, link2, toMask(coeffs))));
}

////////////////////// alex /////////////////////////
void PosCntInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  childFromJson(params, xyz,"xyz");
  childFromJson(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());

  string linkstr;
  childFromJson(params, linkstr, "link");
  link = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link) {
    throw MY_EXCEPTION( (boost::format("invalid link name: %s")%linkstr).str());
  }
}
CntInfoPtr PosCntInfo::create() {
  return CntInfoPtr(new PosCntInfo());
}
void PosCntInfo::hatch(TrajOptProb& prob) {
  VectorXd coeffs(3); coeffs << pos_coeffs;
  prob.addConstr(ConstraintPtr(new CartPosConstraint(prob.GetVarRow(timestep), OR::Vector(xyz[0], xyz[1], xyz[2]), prob.GetRAD(), link, toMask(coeffs))));
}
////////////////////// alex /////////////////////////

void CartVelCntInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, first_step, "first_step");
  childFromJson(params, last_step, "last_step");
  childFromJson(params, distance_limit,"distance_limit");

  string linkstr;
  childFromJson(params, linkstr, "link");
  link = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link) {
    throw MY_EXCEPTION( (boost::format("invalid link name: %s")%linkstr).str());
  }
}
CntInfoPtr CartVelCntInfo::create() {
  return CntInfoPtr(new CartVelCntInfo());
}
void CartVelCntInfo::hatch(TrajOptProb& prob) {
  for (int iStep = first_step; iStep < last_step; ++iStep) {
    prob.addConstr(ConstraintPtr(new CartVelConstraint(prob.GetVarRow(iStep), prob.GetVarRow(iStep+1), prob.GetRAD(), link, distance_limit)));
  }
}

void JointVelCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  childFromJson(params, coeffs,"coeffs");
  int n_dof = gPCI->rad->GetDOF();
  if (coeffs.size() == 1) coeffs = DblVec(n_dof, coeffs[0]);
  else if (coeffs.size() != n_dof) {
    throw MY_EXCEPTION( (boost::format("wrong number of coeffs. expected %i got %i")%n_dof%coeffs.size()).str() );
  }

}
CostInfoPtr JointVelCostInfo::create() {
  return CostInfoPtr(new JointVelCostInfo());
}
void JointVelCostInfo::hatch(TrajOptProb& prob) {
  prob.addCost(CostPtr(new JointVelCost(prob.GetVars(), toVectorXd(coeffs))));
}

void CollisionCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  int n_steps = gPCI->basic_info.n_steps;
  childFromJson(params, coeffs,"coeffs");
  if (coeffs.size() == 1) coeffs = DblVec(n_steps, coeffs[0]);
  else if (coeffs.size() != n_steps) {
    throw MY_EXCEPTION( (boost::format("wrong size: coeffs. expected %i got %i")%n_steps%coeffs.size()).str() );
  }
  childFromJson(params, dist_pen,"dist_pen");
  if (dist_pen.size() == 1) dist_pen = DblVec(n_steps, dist_pen[0]);
  else if (dist_pen.size() != n_steps) {
    throw MY_EXCEPTION( (boost::format("wrong size: dist_pen. expected %i got %i")%n_steps%dist_pen.size()).str() );
  }
}
void CollisionCostInfo::hatch(TrajOptProb& prob) {
  for (int i=0; i < prob.GetNumSteps(); ++i) {
    prob.addCost(CostPtr(new CollisionCost(dist_pen[i], coeffs[i], prob.GetRAD(), prob.GetVarRow(i))));
  }
  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*prob.GetEnv());
  cc->SetContactDistance(*std::max_element(dist_pen.begin(), dist_pen.end()) + .02);
}
CostInfoPtr CollisionCostInfo::create() {
  return CostInfoPtr(new CollisionCostInfo());
}


void ContinuousCollisionCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  int n_steps = gPCI->basic_info.n_steps;
  childFromJson(params, first_step, "first_step", 0);
  childFromJson(params, last_step, "last_step", n_steps-1);
  childFromJson(params, coeffs, "coeffs");
  int n_terms = last_step - first_step;
  cout << "n terms: " << n_terms << endl;
  if (coeffs.size() == 1) coeffs = DblVec(n_terms, coeffs[0]);
  else if (coeffs.size() != n_terms) {
    throw MY_EXCEPTION( (boost::format("wrong size: coeffs. expected %i got %i")%n_terms%coeffs.size()).str() );
  }
  childFromJson(params, dist_pen,"dist_pen");
  if (dist_pen.size() == 1) dist_pen = DblVec(n_terms, dist_pen[0]);
  else if (dist_pen.size() != n_terms) {
    throw MY_EXCEPTION( (boost::format("wrong size: dist_pen. expected %i got %i")%n_terms%dist_pen.size()).str() );
  }
}
void ContinuousCollisionCostInfo::hatch(TrajOptProb& prob) {
  for (int i=first_step; i < last_step; ++i) {
    prob.addCost(CostPtr(new CollisionCost(dist_pen[i], coeffs[i], prob.GetRAD(), prob.GetVarRow(i), prob.GetVarRow(i+1))));
  }
  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*prob.GetEnv());
  cc->SetContactDistance(*std::max_element(dist_pen.begin(), dist_pen.end()) + .02);
}
CostInfoPtr ContinuousCollisionCostInfo::create() {
  return CostInfoPtr(new ContinuousCollisionCostInfo());
}



void JointConstraintInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, vals, "vals");

  int n_dof = gPCI->rad->GetDOF();
  if (vals.size() != n_dof) {
    throw MY_EXCEPTION( (boost::format("wrong number of dof vals. expected %i got %i")%n_dof%vals.size()).str());
  }
  childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
}

void JointConstraintInfo::hatch(TrajOptProb& prob) {
  VarVector vars = prob.GetVarRow(timestep);
  int n_dof = vars.size();
  for (int j=0; j < n_dof; ++j) {
    prob.addLinearConstr(exprSub(AffExpr(vars[j]), vals[j]), EQ);
  }
}
CntInfoPtr JointConstraintInfo::create() {
  return CntInfoPtr(new JointConstraintInfo());
}


}

