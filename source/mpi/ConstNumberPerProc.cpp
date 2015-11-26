#include "ConstNumberPerProc.hpp"
#ifdef RICH_MPI
#include <boost/mpi/communicator.hpp>
#include <boost/mpi/collectives.hpp>
#endif

ConstNumberPerProc::~ConstNumberPerProc(void){}

void ConstNumberPerProc::SetPointsPerProc(double points)
{
	PointsPerProc_=points;
}

ConstNumberPerProc::ConstNumberPerProc(OuterBoundary const& outer,int npercell,
	double speed,double RoundSpeed,int mode):
  outer_(outer),PointsPerProc_(std::max(npercell,1)),speed_(speed),RoundSpeed_(RoundSpeed),
	mode_(mode){}

#ifdef RICH_MPI
void ConstNumberPerProc::Update(Tessellation& tproc,Tessellation const& tlocal)const
{
	const boost::mpi::communicator world;
	int nproc=tproc.GetPointNo();
	const int rank = world.rank();
	vector<double> R(static_cast<size_t>(nproc));
	double dx=0;
	double dy=0;
	for(size_t i=0;i<static_cast<size_t>(nproc);++i)
	  R[i]=tproc.GetWidth(static_cast<int>(i));
	// Make cell rounder
	const Vector2D& CM=tproc.GetCellCM(rank);
	Vector2D point = tproc.GetMeshPoint(rank);
	const double d=abs(CM-tproc.GetMeshPoint(rank));
	double dxround=0,dyround=0;
	if(d>0.1*R[static_cast<size_t>(rank)])
	{
		dxround=RoundSpeed_*speed_*(CM.x-point.x);
		dyround=RoundSpeed_*speed_*(CM.y-point.y);
	}
	point = CM;
	// Find out how many points each proc has
	vector<int> NPerProc(static_cast<size_t>(nproc));
	int mypointnumber=tlocal.GetPointNo()+1;
	boost::mpi::gather(world, mypointnumber, NPerProc, 0);
	boost::mpi::broadcast(world, NPerProc, 0);
	// Move point according to density
	if(mode_==1||mode_==3)
	{
	  for(size_t i=0;i<static_cast<size_t>(nproc);++i)
		{
		  Vector2D otherpoint=tproc.GetCellCM(static_cast<int>(i));
			double dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
					 (point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			double temp=M_PI*pow(R[static_cast<size_t>(rank)],3)*(PointsPerProc_/NPerProc[i]-1)*
			  (point.x-otherpoint.x)/pow(dist,3);
			dx+=temp;
			temp=M_PI*pow(R[static_cast<size_t>(rank)],3)*(PointsPerProc_/NPerProc[i]-1)*
			  (point.y-otherpoint.y)/pow(dist,3);
			dy+=temp;
			// Right side
			otherpoint.x=2*outer_.GetGridBoundary(Right)-otherpoint.x;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Right Up side
			otherpoint.y=2*outer_.GetGridBoundary(Up)-otherpoint.y;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Right Down side
			otherpoint.y=2*outer_.GetGridBoundary(Down)-tproc.GetCellCM(static_cast<int>(i)).y;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Center bottom side
			otherpoint.x=tproc.GetCellCM(static_cast<int>(i)).x;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Left Bottom side
			otherpoint.x=2*outer_.GetGridBoundary(Left)-otherpoint.x;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Left side
			otherpoint.y=tproc.GetCellCM(static_cast<int>(i)).y;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Left top side
			otherpoint.y=2*outer_.GetGridBoundary(Up)-otherpoint.y;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
			// Top side
			otherpoint.x=tproc.GetCellCM(static_cast<int>(i)).x;
			dist=sqrt((point.x-otherpoint.x)*(point.x-otherpoint.x)+
				(point.y-otherpoint.y)*(point.y-otherpoint.y)+0.2*R[static_cast<size_t>(rank)]*R[i]);
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.x-otherpoint.x)/(dist*dist*dist);
			dx+=temp;
			temp=M_PI*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*R[static_cast<size_t>(rank)]*(PointsPerProc_/NPerProc[i]-1)*
				(point.y-otherpoint.y)/(dist*dist*dist);
			dy+=temp;
		}
	}
	double old_dx=dx;
	double old_dy=dy;
	dx=0;
	dy=0;
	// Moving according to pressure
	if(mode_==1||mode_==2)
	{
		const double neigheps = 0.2;
		vector<int> neigh=tproc.GetNeighbors(rank);
		for(size_t i=0;i<neigh.size();++i)
		{
			if(neigh[i]==-1)
				continue;
			Vector2D otherpoint=tproc.GetMeshPoint(neigh[i]);
			//Vector2D otherpoint=tproc.GetCellCM(neigh[i]);
			point = tproc.GetMeshPoint(rank);
			const double dist = tproc.GetMeshPoint(rank).distance(tproc.GetMeshPoint(neigh[i]));
			if (dist<neigheps*std::min(R[static_cast<size_t>(rank)], R[static_cast<size_t>(neigh[i])]))
			{
				dx = neigheps*(point.x - tproc.GetMeshPoint(neigh[i]).x)*std::min(R[static_cast<size_t>(rank)], R[static_cast<size_t>(neigh[i])])/dist;
				dy = neigheps*(point.y - tproc.GetMeshPoint(neigh[i]).y)*std::min(R[static_cast<size_t>(rank)], R[static_cast<size_t>(neigh[i])]) / dist;
			}
			else
			{
				dx -= (NPerProc[static_cast<size_t>(rank)] - NPerProc[static_cast<size_t>(neigh[i])])*(otherpoint.x - point.x)*R[static_cast<size_t>(rank)] / (PointsPerProc_*dist);
				dy -= (NPerProc[static_cast<size_t>(rank)] - NPerProc[static_cast<size_t>(neigh[i])])*(otherpoint.y - point.y)*R[static_cast<size_t>(rank)] / (PointsPerProc_*dist);
			}
		}
	}
	const double FarFraction = 0.2;
	old_dx = (old_dx>0) ? std::min(old_dx, FarFraction*speed_*R[static_cast<size_t>(rank)]) : -std::min(-old_dx, FarFraction*speed_*R[static_cast<size_t>(rank)]);
	old_dy = (old_dy>0) ? std::min(old_dy, FarFraction*speed_*R[static_cast<size_t>(rank)]) : -std::min(-old_dy, FarFraction*speed_*R[static_cast<size_t>(rank)]);
	dx=(dx>0) ? std::min(dx,speed_*R[static_cast<size_t>(rank)]) : -std::min(-dx,speed_*R[static_cast<size_t>(rank)]);
	dy=(dy>0) ? std::min(dy,speed_*R[static_cast<size_t>(rank)]) : -std::min(-dy,speed_*R[static_cast<size_t>(rank)]);
	//if(rank==0)
	//	cout<<"Unlimited dx="<<dx<<" dy="<<dy<<" old_dx="<<old_dx<<" old_dy="<<old_dy<<" max allowed="<<speed_*R[static_cast<size_t>(rank)]<<endl;
	// Combine the two additions
	dx+=old_dx;
	dy+=old_dy;
	// Add the round cells
	dx+=dxround;
	dy+=dyround;
	// Make sure not out of bounds
	point=tproc.GetMeshPoint(rank);
	//	const double close=0.99999;
	const double close = 0.999;
	/*
	const double wx=outer_.GetGridBoundary(Right)-outer_.GetGridBoundary(Left);
	const double wy=outer_.GetGridBoundary(Up)-outer_.GetGridBoundary(Down);
	*/
	const double wx = tproc.GetWidth(rank);
	const double wy = wx;
	const Vector2D center(0.5*(outer_.GetGridBoundary(Left)+outer_.GetGridBoundary(Right)),
			      0.5*(outer_.GetGridBoundary(Up)+outer_.GetGridBoundary(Down)));
	if(point.x+dx>outer_.GetGridBoundary(Right)-(1-close)*wx){
		if(outer_.GetGridBoundary(Right)-point.x<(1-close)*wx)
			dx=-wx*(1-close);
		else
			dx=0.5*(outer_.GetGridBoundary(Right)-point.x);
	}
	if(point.x+dx<outer_.GetGridBoundary(Left)+(1-close)*wx){
		if(-outer_.GetGridBoundary(Left)+point.x<(1-close)*wx)
			dx=wx*(1-close);
		else
			dx=-0.5*(-outer_.GetGridBoundary(Left)+point.x);
	}
	if(point.y+dy>outer_.GetGridBoundary(Up)-(1-close)*wy){
		if(outer_.GetGridBoundary(Up)-point.y<(1-close)*wy)
			dy=-wy*(1-close);
		else
			dy=0.5*(outer_.GetGridBoundary(Up)-point.y);
	}
	if(point.y+dy<outer_.GetGridBoundary(Down)+(1-close)*wy){
		if(-outer_.GetGridBoundary(Down)+point.y<(1-close)*wy)
			dy=wy*(1-close);
		else
			dy=0.5*(outer_.GetGridBoundary(Down)-point.y);
	}
	vector<Vector2D> cor=tproc.GetMeshPoints();
	cor[static_cast<size_t>(rank)]=cor[static_cast<size_t>(rank)]+Vector2D(dx,dy);
	cor.resize(static_cast<size_t>(nproc));
	// Have all processors have the same points
	vector<Vector2D> cortemp(cor.size());
	boost::mpi::gather(world, cor[static_cast<size_t>(rank)], cortemp, 0);
	boost::mpi::broadcast(world, cortemp, 0);
	tproc.Update(cortemp);
}
#endif
