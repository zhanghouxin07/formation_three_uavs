/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;
using namespace ros;
#define N 3
#define UAV_priority_number 0
#define UAV_tar_number 0

//  global var
mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_position;
geometry_msgs::PoseStamped uav1_cur_pos;
std_msgs::Float32MultiArray uav1_tar_ID;
std_msgs::Int8 uav1_prior_ID;
std_msgs::Int16MultiArray uav1_switch_flag,uav1_counter;
float uav1_tar[3];
int uav1_cnt[4]={-1,-1,-1,-1};
int uav1_swfg[3];
geometry_msgs::PoseStamped uav0_cur_pos;
std_msgs::Float32MultiArray uav0_tar_ID;
std_msgs::Int8 uav0_prior_ID;
std_msgs::Int16MultiArray uav0_switch_flag,uav0_counter;
int uav0_cnt[4]={-1,-1,-1,-1};
float uav0_tar[3];
int uav0_swfg[3];
class multiUAV{
    public:

	double Rc       =  2000;  //
	double Rswitch  =  200;  //
	double Rstop    =  20;  //
	double Rattr    =  100;  //
 	double d_des    =  250;  //
	double d0       =  15;  //
	double Arrive = 0.2; //.
	double Zik = 50;    //
	double u   = 0.1;    //

	double pos[N][2];// current position
	double tar[N][2];//destination
	double velo[N][2];
	
	int Counter = 0;
	int i; //priority number

    int Ek[N][N];
    int Dk[N][N];
    int Bk[N][N];
    int Bkp[N][N];
    int Bkpp[N][N];
    int Gk[N][N];
    int switchflag[N][2];
    int comm[N][N];
    int neigh[N][N];

	multiUAV(int i) : i(i) {}
    double dist(double (&a)[2],double (&b)[2])
    {
        int x = a[0] - b[0];	
        int y = a[1] - b[1];
        
        return sqrt(pow(x,2)+pow(y,2));	
    } 

    int any(int a[], int NUM)   //¿ÕŒ¯·µ»Ø1£»·Ç¿Õ·µ»Ø0£» 
    {
            int noDK = 1;
            for (int j=0;j<NUM;j++)
            {
                
                if (a[j] != 888)
                    {
                        noDK = 0;
                        break;
                    }
            }
            return noDK;
    }

    double vectormul(double (&a)[2],double (&b)[2],double (&c)[2],double (&d)[2])
    {
        int x1 = a[0]-b[0];
        int y1 = a[1]-b[1];
        int x2 = c[0]-d[0];
        int y2 = c[1]-d[1];
        return x1*x2 + y1*y2;
    }

    int *solve(double (*pos)[2],double (*tar)[2],int Number){
        Counter = Counter + 1;

		
	
		for(int j=0;j<Number;j++)
		{
			Ek[i][j]=888;
    	 	Dk[i][j]=888;
    	 	Bk[i][j]=888;
    	 	Bkp[i][j]=888;
    	 	Bkpp[i][j]=888;
    	 	Gk[i][j]=888;
    	
    	 	comm[i][j]=888;
    	 	neigh[i][j]=888;
    	 	switchflag[i][0]=888;
			switchflag[i][1]=888;
    	 	
    	 	
		
		}	
		
		for(int j=0;j<Number;j++) //
		{
				if(dist(pos[i],pos[j]) <= Rc && i != j)	
					{
						neigh[i][j] = j;
					}
//					cout<<"see"<<dist(pos[i],pos[j])<<endl;
//					cout<<pos[i][0]<<" "<<pos[i][1]<<endl;
//					cout<<pos[j][0]<<" "<<pos[j][1]<<endl;
		}
		
		for(int j=0;j<Number;j++)  //
		{
			if(dist(pos[i],pos[j]) <= Rswitch && i != j)	
					{
						comm[i][j] = j;
					}
			
		}
		
		for(int j=0;j<Number;j++)  //
		{
			if( neigh[i][j] != 888 )	
				{
				if(dist(pos[i],pos[j]) <= Rstop) 	
					{
						Ek[i][j] = j;
					}		
				}
			
		}
		
		for(int j=0;j<i;j++)  //
		{
			if( neigh[i][j] != 888 )	
				{
				if(dist(pos[i],pos[j]) <= 2*Rstop && dist(pos[j],tar[j])> Rattr ) 	
					{
						Dk[i][j] = j;
					}		
				}	
		}
		
		
		if(dist(pos[i],tar[i]) > Rattr && any(Dk[i],Number) ) //
		{
			
			for (int j=0;j<Number;j++) //
			{
				if (comm[i][j] != 888 )
				{

					if(dist(pos[j],tar[i]) < dist(pos[i],tar[i]))
					 {
					 	Bk[i][j] = j;
					 }
					
				}
			}
		
			for (int j=i+1;j<Number;j++) //
			{
				if (Bk[i][j] != 888 )
				{
//					if(Counter == 5 && i==0)
//					{
//						cout<<"hello"<<endl;
//						cout<<tar[j][0]<<" "<<tar[j][1]<<endl;
//						cout<<dist(pos[j],tar[j])<<endl;
//
//						cout<<"hello"<<endl;
//					}
					if(dist(pos[j],tar[j]) > Rattr)
					 {
					 	Bkp[i][j] = j;
					 }
					
				}
			}
			
			for (int j=0;j<Number;j++) //
			{
				if (Bk[i][j] != 888 )
				{
					if(dist(pos[j],tar[j]) <= Rattr && dist(pos[i],tar[j]) <= Rattr )
					 {
					 	Bkpp[i][j] = j;
					 }
					
				}
			}
			
			

			
			for (int j=i+1;j<Number;j++) // 
			{
				if (Bk[i][j] != 888 )
				{
					for (int k=0;k<Number;k++)	
					{
						if (j != k)
						{
							if(  dist(pos[j],tar[k]) <= Rstop && dist(pos[k],tar[k]) <= Rattr && dist(pos[k],tar[i]) <= dist(pos[i],tar[i]) && dist(pos[k],tar[k]) > dist(pos[j],tar[k])  )
							{
								Gk[i][j] = j;
								break;	
							}
							
						}
						
					}
				
				}
			
			}
			
		}
////////////////////////////////////////////////////////////			
//			if(i==0)
//			{
//					cout<<Counter<<endl;
//					cout<<pos[0][0]<<" "<<pos[0][1]<<endl;
//					cout<<pos[1][0]<<" "<<pos[1][1]<<endl;
//					cout<<pos[2][0]<<" "<<pos[2][1]<<endl;
//					cout<<comm[i][0]<<" "<<comm[i][1]<<" "<<comm[i][2]<<endl;
//					cout<<Bk[i][0]<<" "<<Bk[i][1]<<" "<<Bk[i][2]<<endl;
//					cout<<Bkp[i][0]<<" "<<Bkp[i][1]<<" "<<Bkp[i][2]<<endl;
//					cout<<Bkpp[i][0]<<" "<<Bkpp[i][1]<<" "<<Bkpp[i][2]<<endl;
//					cout<<Gk[i][0]<<" "<<Gk[i][1]<<" "<<Gk[i][2]<<endl;
//					cout<<endl;
//		
//			}
/////////////////////////////////////////////////////////////////////////

		
		if( any(Bkp[i],Number) == 0 || any(Bkpp[i],Number) == 0 ) //rule 2
		{
			double mindistancej = 1000000;

			int jstar = 0;
			for(int j=0;j<Number;j++)
			{
				if(Bkp[i][j] != 888)
				{
					if(dist(pos[j],tar[i]) <= mindistancej)
					{
						jstar = Bkp[i][j];
						mindistancej = dist(pos[j],tar[i]);
					}
				}
			}
			
			for(int j=0;j<Number;j++)
			{
				if(Bkpp[i][j] != 888)
				{
					if(dist(pos[j],tar[i]) <= mindistancej)
					{
						jstar = Bkpp[i][j];
						mindistancej = dist(pos[j],tar[i]);
					}
				}
			}

			
				switchflag[i][0] = 1; 
				switchflag[i][1] = jstar;
			
			
	
		}
		else if(any(Bk[i],Number)==0)  // rule 3
		{
			if(any(Gk[i],Number)==0)
			{
				double mindistanceg = 1000000;
				int gstar = 0;
				for(int j=0;j<Number;j++)
				{
					if(Gk[i][j] != 888)
					{
						if(dist(pos[j],tar[i]) <= mindistanceg)
						{
						gstar = Gk[i][j];
						mindistanceg = dist(pos[j],tar[i]);
						}
					}
				}
				
				switchflag[i][0] = 1; 
				switchflag[i][1] = gstar;				
			}
			
		}
//		if(Counter == 5 && i==0)
//		{
//			cout<<"switch="<<switchflag[i][0]<<" "<<switchflag[i][1]<<endl;
//		}

		int *shuzu = new int[2];
		for(int k=0;k<2;k++)
		{
			shuzu[k] = switchflag[i][k];
		}
		return shuzu;
		
    }
	void exchange(int (*switchflag_use)[2],double (*nowtarget)[2],int Number){
		for(int ll=0;ll<Number;ll++)
		{
			switchflag[ll][0] = switchflag_use[ll][0];
			switchflag[ll][1] = switchflag_use[ll][1];
			tar[ll][0] = nowtarget[ll][0];
			tar[ll][1] = nowtarget[ll][1];
		}
		
		
		for(int ll=0;ll<Number;ll++)
		{
		
		if(switchflag[ll][0] == 1) //È·ï¿œï¿œï¿œï¿œï¿œï¿œï¿œï¿œÏµ
		{
			if(switchflag[switchflag[ll][1]][0]==888)
			{
				switchflag[switchflag[ll][1]][0] = 2;
				switchflag[switchflag[ll][1]][1] = ll;
			}
			else if(switchflag[switchflag[ll][1]][0]==1)
			{
				
				switchflag[ll][0] = 888;
				switchflag[ll][1] = 888;
				
//				if(Counter == 4 && i==0 && ll==0)
//				{
//				cout<<"hello"<<endl;
//				cout<<switchflag[ll][0]<<" "<<switchflag[ll][1]<<endl;
//				cout<<"hello"<<endl;
//				cout<<endl;
//				}
			}
			else if(switchflag[switchflag[ll][1]][0]==2)
			{
				if(ll < switchflag[switchflag[ll][1]][1])
				{
					switchflag[switchflag[ll][1]][1] = ll;
				}
				else
				{
					switchflag[ll][0] = 888;
					switchflag[ll][1] = 888;
				}
			}
		}
	
		}
		
//		if(Counter == 4 && i == 0)
//		{
//			cout<<"hello"<<endl;
//			cout<<switchflag[0][0]<<" "<<switchflag[0][1]<<endl;
//			cout<<switchflag[1][0]<<" "<<switchflag[1][1]<<endl;
//			cout<<switchflag[2][0]<<" "<<switchflag[2][1]<<endl;
//			cout<<tar[1][0]<<" "<<tar[1][1]<<endl;
//			//cout<<tar[i][0]<<" "<<tar[i][1]<<endl;
//			cout<<"hello"<<endl;
//			cout<<endl;
//		}
		



		
		if(switchflag[i][0] == 1 ) //ï¿œï¿œï¿œï¿œï¿œï¿œï¿œË»ï¿œï¿œï¿œï¿œ
		{
			if(switchflag[switchflag[i][1]][1] == i && switchflag[switchflag[i][1]][0] == 2)
			{	

				double change[2];
				//change[0] = pos[i-1][0];
				//change[1] = pos[i-1][1];
				//pos[i-1][0] = pos[(int)switchflag[i-1][2]][0];
				//pos[i-1][1] = pos[(int)switchflag[i-1][2]][1];
				//pos[(int)switchflag[i-1][2]][0] = change[0];
				//pos[(int)switchflag[i-1][2]][0] = change[1];
				change[0] = tar[i][0];
				change[1] = tar[i][1];
				tar[i][0] = tar[switchflag[i][1]][0];
				tar[i][1] = tar[switchflag[i][1]][1];
				tar[switchflag[i][1]][0] = change[0];
				tar[switchflag[i][1]][1] = change[1];
				
			}
		}
		if(switchflag[i][0] == 2 )
		{
			if(switchflag[switchflag[i][1]][1] == i && switchflag[switchflag[i][1]][0] == 1)
			{	

				double change[2];
				//change[0] = pos[i-1][0];
				//change[1] = pos[i-1][1];
				//pos[i-1][0] = pos[(int)switchflag[i-1][2]][0];
				//pos[i-1][1] = pos[(int)switchflag[i-1][2]][1];
				//pos[(int)switchflag[i-1][2]][0] = change[0];
				//pos[(int)switchflag[i-1][2]][0] = change[1];
				change[0] = tar[i][0];
				change[1] = tar[i][1];
				tar[i][0] = tar[switchflag[i][1]][0];
				tar[i][1] = tar[switchflag[i][1]][1];
				tar[switchflag[i][1]][0] = change[0];
				tar[switchflag[i][1]][1] = change[1];
				

				
			}
		}
	
	}
	void veloupdate(double (*pos_use)[2],double(*tar_use)[2], double(*velo_use)[2], int Number, double Zik, double u)
	{
		double Zikuse = Zik;//ï¿œï¿œï¿œï¿œÎ»ï¿œï¿œ

		for(int j=0; j<Number; j++)
		{
			if(i != j)
			{
				if( dist(pos_use[i],pos_use[j]) > Rstop && dist(pos_use[i],pos_use[j]) <= Rstop+2*Zik && vectormul(tar_use[i],pos_use[i],pos_use[i],pos_use[j]) < 0 )
				{
					Zikuse = u * Zik;
					break;
				}
			}
		}
		
		if(dist(pos_use[i],tar_use[i]) <= Zikuse ) //ï¿œï¿œï¿œï¿œÒªï¿œï¿œï¿œï¿œï¿œï¿œ  ï¿œÙ¶Èžï¿œï¿œï¿œ 
		{
			velo_use[i][0] = tar_use[i][0] - pos_use[i][0];
			velo_use[i][1] = tar_use[i][1] - pos_use[i][1]; 
		}
		else 
		{
			velo_use[i][0] = Zikuse * (tar_use[i][0] - pos_use[i][0])/dist(pos_use[i],tar_use[i]);
			velo_use[i][1] = Zikuse * (tar_use[i][1] - pos_use[i][1])/dist(pos_use[i],tar_use[i]);
		}
		
//		if(i == 0){
//		
//		cout<<"zikuse"<<Zikuse<<endl;
//		cout<<"dist"<<dist(pos[i],tar[i])<<endl;
//		cout<<"pos_use"<<pos[i][0]<<" "<<pos[i][1]<<endl;
//		cout<<"tar_use"<<tar[i][0]<<" "<<tar[i][1]<<endl;
//		cout<<"velo_use"<<velo[i][0]<<" "<<velo[i][1]<<endl;
//		}
		
		for(int j=0;j<Number;j++)   //ï¿œÙ¶ï¿œ=0ï¿œï¿œï¿œï¿œï¿œï¿œ 1
		{
			if (i != j )
			{
				if(dist(pos_use[i],pos_use[j]) <= Rstop && dist(pos_use[i],tar_use[i]) > dist(pos_use[j],tar_use[i]) )
				{
					velo_use[i][0] = 0;
					velo_use[i][1] = 0;
					break;
				}
			}
		}
		for(int j=0;j<Number;j++)   //ï¿œÙ¶ï¿œ=0ï¿œï¿œï¿œï¿œï¿œï¿œ 2
		{
			if (Ek[i][j] != 888)
			{
				if(dist(pos_use[i],tar_use[j]) > dist(pos_use[j],tar_use[j]) && i>j && dist(pos_use[j],tar_use[j]) > Rattr && vectormul(tar_use[j],pos_use[j],pos_use[i],pos_use[j]) > 0 )
				{
					velo_use[i][0] = 0;
					velo_use[i][1] = 0;
					break;
				}
				else if(dist(pos_use[i],tar_use[j]) > dist(pos_use[j],tar_use[j]) && i>j && dist(pos_use[j],tar_use[j]) > Rattr && vectormul(tar_use[i],pos_use[i],pos_use[j],pos_use[i]) > 0)
				{
					velo_use[i][0] = 0;
					velo_use[i][1] = 0;
					break;
				}
				else if( dist(pos_use[i],tar_use[j]) < dist(pos_use[j],tar_use[j]) && dist(pos_use[i],tar_use[i]) <= Rattr-Rstop && vectormul(tar_use[i],pos_use[i],pos_use[j],pos_use[i]) > 0 )
				{
					velo_use[i][0] = 0;
					velo_use[i][1] = 0;
					break;
				}
						
			}
		}
		

	    
		pos[i][0] = pos_use[i][0] + velo_use[i][0];
		pos[i][1] = pos_use[i][1] + velo_use[i][1];
		
//		if(i==0)
//		{
//		cout<<"velo"<<velo_use[i][0]<<" "<<velo_use[i][1]<<endl;
//		cout<<"pos"<<pos[i][0]<<" "<<pos[i][1]<<endl;
//		cout<<endl;
//	    }
	}
};
//callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}
void uav1_cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_cur_pos = *msg;
}
void uav1_tar_ID_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	uav1_tar_ID = *msg;
	for(auto i=0;i<3;i++)uav1_tar[i]=uav1_tar_ID.data[i];
}
void uav1_prior_ID_cb(const std_msgs::Int8::ConstPtr& msg){
	uav1_prior_ID = *msg;
}
void uav1_switch_flag_cb(const std_msgs::Int16MultiArray::ConstPtr& msg){
	uav1_switch_flag = *msg;
	for(int i=0;i<3;i++) uav1_swfg[i]=uav1_switch_flag.data[i];
}
void uav1_counter_cb(const std_msgs::Int16MultiArray::ConstPtr& msg){
	uav1_counter = *msg;
	for(int i=0;i<3;i++) uav1_cnt[i]=uav1_counter.data[i];
}
void uav0_cur_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav0_cur_pos = *msg;
}
void uav0_tar_ID_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	uav0_tar_ID = *msg;
	for(auto i=0;i<3;i++)uav0_tar[i]=uav0_tar_ID.data[i];
}
void uav0_prior_ID_cb(const std_msgs::Int8::ConstPtr& msg){
	uav0_prior_ID = *msg;
}
void uav0_switch_flag_cb(const std_msgs::Int16MultiArray::ConstPtr& msg){
	uav0_switch_flag = *msg;
	for(int i=0;i<3;i++) uav0_swfg[i]=uav0_switch_flag.data[i];
}
void uav0_counter_cb(const std_msgs::Int16MultiArray::ConstPtr& msg){
	uav0_counter = *msg;
	for(int i=0;i<3;i++) uav0_cnt[i]=uav0_counter.data[i];
}
void uav_tar_init(multiUAV &multiUAV){
	vector<float> tar;
	tar.push_back(500);tar.push_back(500);
	tar.push_back(250);tar.push_back(500);
	tar.push_back(0);tar.push_back(500);
	for(auto i=0; i<3;i++){
		multiUAV.tar[i][0]=tar[2*i];
		multiUAV.tar[i][1]=tar[2*i+1];
		// cout<<tar[2*i]<<' '<<tar[2*i]<<endl;
	}
}
struct local_error
{
	float x;
	float y;
	float z;
	local_error(float xx,float yy,float zz){x=xx;y=yy;z=zz;}
};
float dis(float a,float b,float c, float d){
	float m = (a-b)*(a-b);
	float n = (c-d)*(c-d);
	return sqrt(m+n);
}
int main(int argc, char **argv)
{
	// algorithm
	multiUAV multiUAV0(0);
	multiUAV multiUAV1(1);
	multiUAV multiUAV2(2);
	// destination init
	uav_tar_init(multiUAV0);uav_tar_init(multiUAV1);uav_tar_init(multiUAV2);
	local_error le(500,0,0);
    ros::init(argc, argv, "offb_node2");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav2/mavros/state", 10, state_cb);
	ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("/uav2/mavros/local_position/pose",10,local_position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");

	// algotithm communication
	// boradcast self position
	ros::Publisher cur_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/pos/global", 10);
	
	// boradcast tar ID;
	ros::Publisher tar_ID_pub = nh.advertise<std_msgs::Float32MultiArray>
            ("/uav2/tar_ID", 10);
	
	// boradcast priority ID;
	ros::Publisher prior_ID_pub = nh.advertise<std_msgs::Int8>
            ("/uav2/prior_ID", 10);
	
	// boradcast switch_flag;
	ros::Publisher switch_flag_pub = nh.advertise<std_msgs::Int16MultiArray>
            ("/uav2/switch_flag", 10);
	// boradcast counter;
	ros::Publisher counter_pub = nh.advertise<std_msgs::Int16MultiArray>
            ("/uav2/counter", 10);
	geometry_msgs::PoseStamped cur_pos;
	std_msgs::Float32MultiArray tar_ID;
	std_msgs::Int8 prior_ID;
	std_msgs::Int16MultiArray switch_flag,counter;
	// [0] is main cnt, [1] is pos and tar cnt ,[2] is exchange cnt
	for(auto i=0;i<3;i++) counter.data.push_back(-1);
	prior_ID.data = UAV_priority_number;
	// end
	// Reveive others' info 
	// uav1
	ros::Subscriber uav1_cur_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/pos/global", 10, uav1_cur_pos_cb);
	ros::Subscriber uav1_tar_ID_sub = nh.subscribe<std_msgs::Float32MultiArray>
			("/uav1/tar_ID",10, uav1_tar_ID_cb);
	ros::Subscriber uav1_prior_ID_sub = nh.subscribe<std_msgs::Int8>
			("/uav1/prior_ID",10, uav1_prior_ID_cb);
	ros::Subscriber uav1_switch_flag_sub = nh.subscribe<std_msgs::Int16MultiArray>
			("/uav1/switch_flag",10, uav1_switch_flag_cb);
	ros::Subscriber uav1_counter_sub = nh.subscribe<std_msgs::Int16MultiArray>
			("/uav1/counter",10, uav1_counter_cb);
	// uav0
	ros::Subscriber uav0_cur_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav0/pos/global", 10, uav0_cur_pos_cb);
	ros::Subscriber uav0_tar_ID_sub = nh.subscribe<std_msgs::Float32MultiArray>
			("/uav0/tar_ID",10, uav0_tar_ID_cb);
	ros::Subscriber uav0_prior_ID_sub = nh.subscribe<std_msgs::Int8>
			("/uav0/prior_ID",10, uav0_prior_ID_cb);
	ros::Subscriber uav0_switch_flag_sub = nh.subscribe<std_msgs::Int16MultiArray>
			("/uav0/switch_flag",10, uav0_switch_flag_cb);
	ros::Subscriber uav0_counter_sub = nh.subscribe<std_msgs::Int16MultiArray>
			("/uav0/counter",10, uav0_counter_cb);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2.5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
	int main_cnt=0, sec_cnt=0;
    while(ros::ok()){
        auto st=clock();
		float store[20]={0};
		while(!arm_cmd.response.success&&(clock()-st)/CLOCKS_PER_SEC<60.0){
			if(!ros::ok()) break;
			if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
			} else {
				if( !current_state.armed &&
					(ros::Time::now() - last_request > ros::Duration(5.0))){
					if( arming_client.call(arm_cmd) &&
						arm_cmd.response.success){
						ROS_INFO("Vehicle armed");
					}
					last_request = ros::Time::now();
				}
        }
			local_pos_pub.publish(pose);
		}
		
		// get self position info
		cout<<"local_position:"<<local_position.pose.position.x<<' '<<local_position.pose.position.y<<' '<<local_position.pose.position.z<<endl;
		ROS_INFO("wait %d, hold on",main_cnt);

		// data update
		// main_cnt sync
		counter.data[0]=main_cnt;
		counter_pub.publish(counter);
		// judge time sync
		if(sec_cnt==0) 
		if(uav1_cnt[0]==main_cnt && uav0_cnt[0]==main_cnt) sec_cnt=1;

		// cur and pos sync
		int uav1_p_cnt,uav0_p_cnt,uav1_t_cnt,uav0_t_cnt;
		// communication for sending pos
		cur_pos.pose.position.x=local_position.pose.position.x+le.x;
		cur_pos.pose.position.y=local_position.pose.position.y+le.y;
		cur_pos.pose.position.z=local_position.pose.position.z+le.z;
		cur_pos.pose.orientation.x=main_cnt;
		cur_pos_pub.publish(cur_pos);
		// communication for getting pos
		multiUAV2.pos[0][0]=uav0_cur_pos.pose.position.x;
		multiUAV2.pos[0][1]=uav0_cur_pos.pose.position.y;
		uav0_p_cnt=uav0_cur_pos.pose.orientation.x;
		multiUAV2.pos[1][0]=uav1_cur_pos.pose.position.x;
		multiUAV2.pos[1][1]=uav1_cur_pos.pose.position.y;
		uav1_p_cnt=uav1_cur_pos.pose.orientation.x;
		multiUAV2.pos[2][0]=cur_pos.pose.position.x;
		multiUAV2.pos[2][1]=cur_pos.pose.position.y;
		
		store[0]=multiUAV2.pos[0][0];
		store[1]=multiUAV2.pos[0][1];
		store[2]=multiUAV2.pos[1][0];
		store[3]=multiUAV2.pos[1][1];
		store[4]=multiUAV2.pos[2][0];
		store[5]=multiUAV2.pos[2][1];
		// for tar
		tar_ID.data.push_back(multiUAV2.tar[2][0]);
		tar_ID.data.push_back(multiUAV2.tar[2][1]);
		tar_ID.data.push_back(main_cnt);
		tar_ID_pub.publish(tar_ID);
		tar_ID.data.clear();
		multiUAV2.tar[1][0]=uav1_tar[0];
		multiUAV2.tar[1][1]=uav1_tar[1];
		uav1_t_cnt=uav1_tar[2];
		multiUAV2.tar[0][0]=uav0_tar[0];
		multiUAV2.tar[0][1]=uav0_tar[1];
		uav0_t_cnt=uav0_tar[2];
		store[6]=multiUAV2.tar[0][0];
		store[7]=multiUAV2.tar[0][1];
		store[8]=multiUAV2.tar[1][0];
		store[9]=multiUAV2.tar[1][1];
		store[10]=multiUAV2.tar[2][0];
		store[11]=multiUAV2.tar[2][1];

		cout<<uav0_p_cnt<<' '<<uav1_p_cnt<<' '<<uav0_t_cnt<<' '<<uav1_t_cnt<<endl;
		// judge pos and tar sync
		int mysolve[2];
		if(sec_cnt==1)
		if(uav1_p_cnt==main_cnt && uav0_p_cnt==main_cnt 
				&& uav1_t_cnt==main_cnt && uav0_t_cnt==main_cnt){
		/*
		* 	solve with all information
		*/
			int * _mysolve = multiUAV2.solve(multiUAV2.pos,multiUAV2.tar,N);
			mysolve[0]=_mysolve[0];
			mysolve[1]=_mysolve[1];
			sec_cnt=2;
		}
		int allsolve[N][2];

		// switch flag sync
		// communication for sending switch_flag
		switch_flag.data.push_back(mysolve[0]);
		switch_flag.data.push_back(mysolve[1]);
		switch_flag.data.push_back(main_cnt);
		switch_flag_pub.publish(switch_flag);
		switch_flag.data.clear();
		// communication for getting switchflag 
		allsolve[0][0] = uav0_swfg[0];
		allsolve[0][1] = uav0_swfg[1];
		int uav0_swfg_cnt = uav0_swfg[2];
		allsolve[1][0] = uav1_swfg[0];
		allsolve[1][1] = uav1_swfg[1];
		int uav1_swfg_cnt = uav1_swfg[2];
		allsolve[2][0] = mysolve[0];
		allsolve[2][1] = mysolve[1];
		
		store[12]=allsolve[0][0];
		store[13]=allsolve[0][1];
		store[14]=allsolve[1][0];
		store[15]=allsolve[1][1];
		store[16]=allsolve[2][0];
		store[17]=allsolve[2][1];
		int tmp_destination_x,tmp_destination_y;
		if(sec_cnt==2)
		if(uav1_swfg_cnt==main_cnt && uav0_swfg_cnt==main_cnt){
			multiUAV2.exchange(allsolve,multiUAV2.tar,N); 
			// control signal and the  multiUAV2.pos will be update in veloupdate
			multiUAV2.veloupdate(multiUAV2.pos,multiUAV2.tar, multiUAV2.velo, N, multiUAV2.Zik, multiUAV2.u);
			tmp_destination_x=multiUAV2.pos[0][0]-le.x;
			tmp_destination_y=multiUAV2.pos[0][1]-le.y;
			sec_cnt=3;
		}

		// Control 
		if(sec_cnt==3){
			cout<<"the uav is moving to next position"<<endl;
			cout<<tmp_destination_x<<' '<<tmp_destination_x<<endl;
			pose.pose.position.x=tmp_destination_x;
			pose.pose.position.y=tmp_destination_y;
			float tmp=dis(tmp_destination_x,local_position.pose.position.x,
					tmp_destination_y,local_position.pose.position.y);
			if(tmp<1.0) {
				sec_cnt=0;
				main_cnt++;
			}
		}
		local_pos_pub.publish(pose);

		// Print store info
		cout<<"pos"<<endl;
		for(int i=0;i<6;i++) cout<<store[i]<<' ';
		cout<<endl;
		cout<<"tar"<<endl;	
		for(int i=6;i<12;i++) cout<<store[i]<<' ';
		cout<<endl;
		cout<<"allsolve"<<endl;
		for(int i=12;i<18;i++) cout<<store[i]<<' ';
		cout<<endl;

		// Spin once 
		cout<<"main_cnt "<<main_cnt<<" sec_cnt "<<sec_cnt<<endl<<endl;
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
