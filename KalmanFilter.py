import numpy as np
import math
import time, msvcrt
import socket


d = 12; #Wheel to Wheel length of the car, in mm
rw = 12; #radius of wheel in mm
keypressTimeout = 3;#timeout to continue running code without keypress input
L = 100;#box length, mm
W = 100;#box width, mm

A = np.eye((4)).astype(float);
B = np.zeros((4,2)).astype(float);
C = np.eye((4)).astype(float);

#NOISE CHARACTERIZATION
#system noise
#modify Q to reflect the covariance in the system error
#diagonals are variance in data
#non diagonals are covariances in data
Q = np.array([[0,0,0,0],
              [0,0,0,0],
              [0,0,1,0],
              [0,0,0,1.0]]);
    
#sensor noise
#modify R to reflect the covariance in the sensor error
#diagonals are variance in data
#non diagonals are covariances in data
R = np.array([[1,0,   0   ,    0],
              [0,1,   0   ,    0],
              [0,0,0.00865,    0],#variance in theta
              [0,0,   0   ,2.460751]]);#variance in theta dot




v = 0;
w = 0;

#null positions
x_null = np.array([[1.0],  #X position init
                   [1.0],  #Y position init
                   [0.0],  #Theta init
                   [0.0]]);#Theta dot init (0)

class StateEstimate:
    def __init__(self):
        self.x_hat = x_null; #initialize class to null position
        self.P = np.zeros((4,4)); #initialize variance to 0
    def updateState(self, x_hat_new, P_new):
        self.x_hat = x_hat_new;
        self.P = P_new;

class robitData:
    
    def __init__(self):
        self.front = 0;
        self.side = 0;
        self.gyrox = 0;
        self.gyroy = 0;
        self.gyroz = 0;
        self.magx = 0;
        self.magy = 0;
        self.magz = 0;

#changable variables
currentState = StateEstimate();
z_t = np.zeros((4,1)); #4x1 state measurement
u_t = np.zeros((2,1));#1x2 input
dT = 1; #time step in milliseconds, initially at zero, to be calculated upon data recieved


def linearize():# use this to calculate the jacobian on the fly, if needed
    pass;

def calcA(wr, wl, theta, deltaT, A):#calculates the entries of A, as defined above
    A[2,0] = -(deltaT*rw*math.sin(theta))/2*(wr+wl);
    A[2,1] = (deltaT*rw*math.cos(theta))/2*(wr+wl);
    A[3,2] = deltaT;
    print(A)
    return A;
    
def calcB(wr, wl, theta, deltaT, B):#calculates the entries of B, as defined above
    B[0,0] = (deltaT*rw*math.cos(theta))/2;
    B[1,0] = (deltaT*rw*math.sin(theta))/2;
    B[2,0] = (deltaT*rw)/d;
    B[3,0] = rw/d;
    B[0,1] = (deltaT*rw*math.cos(theta))/2;
    B[1,1] = (deltaT*rw*math.sin(theta))/2;
    B[2,1] = -(deltaT*rw)/d;
    B[3,1] = -rw/d;
    print(B)
    return B;




def kalmanFilter(x_hat_prev, P_prev, u, z_t):
    #have previous state +, input vector u
    #find predicted state - from input and current state
    x_hat_t_minus = np.matmul(A, x_hat_prev) + np.matmul(B, u);
   
    #find variance of predicted state -
    P_t_minus = np.matmul(np.matmul(A,P_prev),A.T) + Q;
 
    
    #get sensor measurement
    #calculate kalman gain 
    Kt = kalmanGain(P_t_minus);
  
    #predict current state +
    x_hat_t_plus = x_hat_t_minus + np.matmul(Kt,(z_t - np.matmul(C, x_hat_t_minus)));

    #predict variance of predicted state +
    P_t_plus = np.matmul((1-np.matmul(Kt,C)),P_t_minus);

    state = StateEstimate();
    state.updateState(x_hat_t_plus, P_t_plus);
    return state;
    
def kalmanGain(P_t_minus):

    CPC_trans = np.linalg.inv(np.matmul(C, np.matmul(P_t_minus,C.T))+R);

    inv = np.linalg.inv(np.matmul(C, np.matmul(P_t_minus,C.T))+R);
  
    Kt = np.matmul(P_t_minus,np.matmul(C.T,inv));

    return Kt;

def getData():
    pass;
def parseLIDAR(front, side, x, y, theta):
    fdist1x = L - front*math.cos(theta);
    fdist1y = fdist1x * math.tan(theta);
    fdist2y = W - front*math.sin(theta); 
    fdist2x = fdist2y/math.tan(theta);
    fdist3x = front * math.cos(theta + 180); 
    fdist3y = fdist3x * math.tan(theta);
    fdist4y = front*math.sin(theta + 180);
    fdist4x = fdist4y/math.tan(theta);
    
    
    sdist1x = L - side*math.cos(theta);
    sdist1y = sdist1x * math.tan(theta);
    sdist2y = W - side*math.cos(theta + 180); 
    sdist2x = sdist2y/math.tan(theta);
    sdist3x = side * math.sin(theta + 180); 
    sdist3y = sdist3x * math.tan(theta);
    sdist4y = side*math.cos(theta + 180);
    sdist4x = sdist4y/math.tan(theta);
    
    #find the minimum of the above calculated xy pairs
    
    xdiff = np.array([abs(x-fdist1x),abs(x-fdist2x),abs(x-fdist3x),abs(x-fdist4x),abs(x-sdist1x),abs(x-sdist2x),abs(x-sdist3x),abs(x-sdist4x)]);
    ydiff = np.array([abs(y-fdist1y),abs(y-fdist2y),abs(y-fdist3y),abs(y-fdist4y),abs(y-sdist1y),abs(y-sdist2y),abs(y-sdist3y),abs(y-sdist4y)]);
    xvals = np.array([fdist1x,fdist2x,fdist3x,fdist4x,sdist1x,sdist2x,sdist3x,sdist4x]);
    yvals = np.array([fdist1x,fdist2x,fdist3x,fdist4x,sdist1x,sdist2x,sdist3x,sdist4x]);
    xout = 0;
    yout = 0;
    done = 0;
    while(done == 0):
        xmin = np.argmin(fxdiff);
        ymin = np.argmin(fydiff);
        
        if(xmin == ymin):
            xout = xvals[xmin];
            yout = yvals[ymin];
            done = 1;
        else:
            xdiff[xmin] = L;#set the difference high to null it's effect and try again
            ydiff[ymin] = L;
            
            
    return(xout, yout);
    


    
    pass;
    
def setup():
    global A;
    global B;
    global C;
    
    #STATE SPACE REPRESENTATION
    
    #define the dynamics matrix A
    #   [              1              ,              0              , 0  , 0 ]
    #   [              0              ,              1              , 0  , 0 ]
    #   [-(dT*rw*sin(theta))/2(wr+wl) , (dT*rw*cos(theta))/2(wr+wl) , 1  , 0 ]
    #   [              0              ,              0              , dT , 1 ]
    #define the element in the third row, fourth column as dT, the time difference between state updates
    A = calcA(0, 0, currentState.x_hat[2], dT, A);
    
    #define the input matrix B
    #   [(dT*rw*cos(theta))/2 , (dT*rw*cos(theta))/2 ]
    #   [(dT*rw*sin(theta))/2 , (dT*rw*sin(theta))/2 ]
    #   [     (dT*rw)/d       ,      -(dT*rw)/d      ]
    #   [      rw/d           ,         -rw/d        ]
    B = calcB(0, 0, currentState.x_hat[2], dT, B);

      
    #define the C matrix
    #   [1 , 0 , 0 , 0 ]
    #   [0 , 1 , 0 , 0 ]
    #   [0 , 0 , 1 , 0 ]
    #   [0 , 0 , 0 , 1 ]
    C=C;
    #NOISE CHARACTERIZATION
   

    pass;


def main():
    setup();
    
    print(np.array([[100],[100],[66],[0]]))
    
    stateEstimate = kalmanFilter(np.array([[100],[100],[66],[0]]), np.zeros((4,4)),np.array([[0],[0]]), np.array([[99],[101],[66],[5]]))
    
    print(stateEstimate.x_hat)
    print( stateEstimate.P)
    
    
    
    
    
    
    
    global dT;
    global currentState;
    global u_t;
    global z_t;
    sock = socket.socket();
    print("Welcome to this Robit Control Code (^_^)")
    
    lastCMD = "TO";
    cmd = None;
    spd = 0;
    while 1 == 1:
        #generate robot command
        timeoutBegin = time.time();
        while True:
            if msvcrt.kbhit():
                lastCMD = cmd;
                cmd = msvcrt.getch().decode();
                break;
            elif time.time() - timeoutBegin > keypressTimeout:
                lastCMD = cmd;
                cmd = "TO";
                break;
                
                
                
        if cmd == "w":#forward
            if cmd != lastCMD:
                print("f")
                u_t[0] = 90 + 1*spd;
                u_t[1] = 90 + -1*spd;
            pass;
        elif cmd == "s":#reverse
            if cmd != lastCMD:
                print("b");
                u_t[0] = 90 + -1*spd;
                u_t[1] = 90 + 1*spd;
            pass;
        elif cmd == 'a':#left turn
            if cmd != lastCMD:
               print("l")
               u_t[0] = 90 + 1*spd;
               u_t[1] = 90 + 1*spd;
            pass;
        elif cmd == 'd':#right turn
            if cmd != lastCMD:
                print("r")
                u_t[0] = 90 + -1*spd;
                u_t[1] = 90 + -1*spd;
            pass;
        elif cmd == "e":#SPEED UP
            if spd < 90:
                spd+=5;
            cmd = lastCMD;
            print("Increase speed to:")
            print(spd);
            pass;
        elif cmd == "q":#SLOW DOWN
            if spd > 0:
                spd-=5;
            cmd = lastCMD;
            print("Decrease speed to:")
            print(spd);
            pass;
        else: #timeout or invalid input
            if cmd != lastCMD:
                print("none")
            pass;
        
            
        #send robot command
        #send the cmd to the robot
        #send cmd, and spd, both of which the robot should use to determine
        #it's next move. 
            #Using information from the socket tutorial from 
            #https://techtutorialsx.com/2017/11/13/esp32-arduino-setting-a-socket-server/
        sock.connect(('192.168.4.1',80));#connect to the socket
        toSend = cmd + "," + str(spd);
        sock.send(toSend.encode());
        
        #at this point, we've sent to the socket the message toSend, of the form "cmd,spd"
        
        #ask for robot state measurement
        data = b'';
        temp = b'';
        while temp != "T":
           temp = sock.recv(1);
           data += temp;
        sock.close();
        
        
        #data is of form "dt,front,side,gyrox,y,z,magx,y,zT"        
        
        
        
        
        #process data
        #the array data now contains a csv, terminated with a T, of all the data we want from the robot
        #find X, Y, Theta, Theta_dot from data.
        d = robitData();
        #dT
        data = data.decode();
        i = 0;
        if(len(data)>3):
            while i < len(data):#populate dT
                if(i != ','):
                    dT += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
                    
            while i < len(data):#populate lidar
                if(i != ','):
                    d.front += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
            while i < len(data):
                if(i != ','):
                    d.side += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
            
            while i < len(data):#populate gyro data
                if(i != ','):
                    d.gyrox += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
            while i < len(data):
                if(i != ','):
                    d.gyroy += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
            while i < len(data):
                if(i != ','):
                    d.gyroz += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
                    
                    
            while i < len(data):#populate magnetometer data
                if(i != ','):
                    d.magx += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
            while i < len(data):
                if(i != ','):
                    d.magy += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
            while i < len(data):
                if(i != 'T'):
                    d.magz += data[i];
                    i+=1;
                else:
                    i+=1;
                    break;
        #d now contains all the individual data points for the code!
        front = 0.0;
        side = 0.0;#to be used for front and side lidar values
        theta_mag = 0.0;
        theta_dot_gyro = 0.0;
        
        #TODO: parse the data recieved from the robit to figure out the state measurement
        newx, newy = parseLIDAR(front, side, currentState.x_hat[0], currentState.x_hat[1], currentState.x_hat[2]);
        
        
        #State Update!
        z_t[1] = newx;
        z_t[2] = newy;
        z_t[3] = 1;
        z_t[4] = 1;
        
        
        
        
        #kalman filter
        currentState = kalmanFilter(currentState.x_hat, currentState.P, u_t, z_t);
        #report estimated state
        
        #TODO: state estimate reporting
pass;
        
    
if __name__ == "__main__":
     main();
     print("donezo");