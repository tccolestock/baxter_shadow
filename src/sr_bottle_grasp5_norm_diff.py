#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import String, Float32, UInt8
# from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_msgs.msg import BiotacAll
import time
import numpy as np # for: exp(), .shape
import numpy.matlib as npm # for: npm.repmat()


# current revision uses 15 point moving average

rospy.init_node("sr_grasp", anonymous=True)

# arm_commander = SrArmCommander()
hand_commander = SrHandCommander()
time.sleep(1)

class Slip(object):
    """docstring for Slip"""
    def __init__(self):
        self.last3 = [0]*15
        self.mvavg = 0
        self.base = [] #np.matrix( np.zeros((26,1)) )
        self.flag = 0
        self.window = np.matrix([[0]*15]*26)
        self.winavg = np.matrix([[0]]*26)
        self.wincount = 0
        self.winsize = 15

    def new(self, x):
        self.last3.pop()
        self.last3.insert(0,x)
        self.update()

    def update(self):
        self.mvavg = sum(self.last3)/len(self.last3)

    def rebase(self, x):
        self.base = x
        self.flag = 1

    def addpoint(self, x): # needs to receive a column matrix
        # x = np.matrix(x)
        l = self.window.shape[1]
        # Move everything over 1 index -- first in first out
        for i in range(1,l):
            self.window[:,l-i] = self.window[:,l-(i+1)]
        self.window[:,0] = x
        if self.wincount < self.winsize:
            self.wincount = self.wincount + 1
        self.winupdate()

    def winupdate(self):
        l = self.window.shape[0]
        for i in range(l):
            self.winavg[i] = np.mean(self.window[i,:])



# hand positions:::
start = { \
            'rh_FFJ1': -0.013387468694274651,   'rh_FFJ2': 0.10550124582950798,     \
            'rh_FFJ3': -0.07913645703418956,    'rh_FFJ4': -0.020790969983510318,   \
            'rh_THJ4': 0.8987090669167258,      'rh_THJ5': -1.0529838245665772,     \
            'rh_THJ1': 0.36613957472880915,     'rh_THJ2': -0.3099264451304632,     \
            'rh_THJ3': 0.04339213288734181,     'rh_LFJ2': 0.31856120196799154,     \
            'rh_LFJ3': -0.13247924347682977,    'rh_LFJ1': 0.020856552138779016,    \
            'rh_LFJ4': 0.006156109478006114,    'rh_LFJ5': 0.030368858695598477,    \
            'rh_RFJ4': -0.017502072148899307,   'rh_RFJ1': 0.04862574836081379,     \
            'rh_RFJ2': 0.23106641618794493,     'rh_RFJ3': -0.040169677117662395,   \
            'rh_MFJ1': 0.0061621824517631985,   'rh_MFJ3': -0.03814186780706377,    \
            'rh_MFJ2': 0.28535536916148746,     'rh_MFJ4': 0.005735133335643892,    \
            'rh_WRJ2': -0.08740126759572807,    'rh_WRJ1': -0.009642963029241673    \
        }

close = { \
            'rh_FFJ1': 0.5366228138727492,      'rh_FFJ2': 1.3707472622836295,      \
            'rh_FFJ3': 0.6104890181588297,      'rh_FFJ4': -0.1693188654196813,     \
            'rh_THJ4': 1.1494816044032174,      'rh_THJ5': -0.25236240595266746,    \
            'rh_THJ1': 1.0564478227578378,      'rh_THJ2': 0.5591902548242037,      \
            'rh_THJ3': 0.3010860128238289,      'rh_LFJ2': 1.1510589476677358,      \
            'rh_LFJ3': 0.3496450123403709,      'rh_LFJ1': 0.2812655031286765,      \
            'rh_LFJ4': 0.0007317935784767475,   'rh_LFJ5': 0.038378063907728126,    \
            'rh_RFJ4': -0.030822436892029084,   'rh_RFJ1': 0.2252787835450361,      \
            'rh_RFJ2': 1.1696882711839942,      'rh_RFJ3': 0.6358242015720096,      \
            'rh_MFJ1': 0.18990725919524606,     'rh_MFJ3': 0.6792600589796994,      \
            'rh_MFJ2': 1.3251573950327318,      'rh_MFJ4': -0.007377111269187729,   \
            'rh_WRJ2': -0.103164843927744,      'rh_WRJ1': -0.10998772922135532     \
        }

soft_open = { \
                'rh_FFJ1': 0.015299964222028228,    'rh_FFJ2': 1.0363475765685581,      \
                'rh_FFJ3': 0.2156981911673815,      'rh_FFJ4': -0.09041898402453244,    \
                'rh_THJ4': 1.1566064166609298,      'rh_THJ5': -0.4976068025062665,     \
                'rh_THJ1': 0.7331455374652653,      'rh_THJ2': 0.24076301002605377,     \
                'rh_THJ3': 0.2482866853523483,      'rh_LFJ2': 0.9579282517503304,      \
                'rh_LFJ3': 0.22891073506641474,     'rh_LFJ1': 0.0369458923601228,      \
                'rh_LFJ4': -0.010122565656606665,   'rh_LFJ5': 0.03884889211514442,     \
                'rh_RFJ4': -0.03515217103578468,    'rh_RFJ1': 0.06709122242188231,     \
                'rh_RFJ2': 0.8408973912178247,      'rh_RFJ3': 0.34325412649756837,     \
                'rh_MFJ1': 0.014565158522349297,    'rh_MFJ3': 0.4407150002695516,      \
                'rh_MFJ2': 0.7245574605990543,      'rh_MFJ4': -0.005447683603367941,   \
                'rh_WRJ2': -0.106417846398269,      'rh_WRJ1': -0.07804339747071865     \
            }

def callback3(data):
    features = list(data.tactiles[0].electrodes) # comes in as a Tuple, convert to list
    features.append(data.tactiles[0].pac1) # append the Pac1 value
    features = np.matrix(features) # convert list to numpy matrix
    angle = nnfittingtest02result(features.T) # transpose matrix to create column vector(s)
    slip.new(angle)
    print("slip: %f" %slip.mvavg)
    if (slip.mvavg > 85) and (slip.mvavg < 95):
        print("Upward slip detected!")
        hand_commander.move_to_joint_value_target_unsafe(start, 1, True)
        time.sleep(1)
        rospy.signal_shutdown("Slip was Detected")

def callback4(data):
    realtime = list(data.tactiles[0].electrodes) # comes in as a Tuple, convert to list
    realtime.append(data.tactiles[0].pac1) # append the Pac1 value
    realtime.append(data.tactiles[0].pdc)
    # if slip.flag == 0:
    #     slip.rebase(realtime)
    features = np.array([0]*26)
    s = sum(realtime[:24])
    s = s/(100*1000) # *100 to make it a %, *1000 to scale it to same mag as pressure
    # print(s)
    features[:24] = np.array(realtime[:24])/s
    features[24:] = realtime[24:]
    features = np.matrix(features) # convert list to numpy matrix
    # print("1")
    # print(features)
    if slip.wincount < slip.winsize:
        slip.addpoint(features.T)
        print("Building window...")
    else:
        # print("window::")
        # print(slip.window[:])
        # print("2")
        # print(features)
        fv = features.T - slip.winavg[:]
        # print("winavg")
        # print(slip.winavg[:])
        # print("3")
        # print(features)
        # time.sleep(5)
        angle = nnfittingtest02result(fv) # transpose matrix to create column vector(s)
        if angle > 360:
            angle = 180
        slip.new(angle)
        slip.addpoint(features.T)
        # print("slip: %f ------ last3: %s" %(slip.mvavg, slip.last3))
        print("slip: %f" %slip.mvavg)

        if (slip.mvavg > 170) and (slip.mvavg < 190):
            print("Upward slip detected!")
            print("features")
            print(features[:])
            print("window")
            print(slip.window[:])
            print("window average")
            print(slip.winavg[:])
            print("last3")
            print(slip.last3[:])
            hand_commander.move_to_joint_value_target_unsafe(start, 1, True)
            time.sleep(1)
            rospy.signal_shutdown("Slip was Detected")

#######################################################

# Based on nnfittingtest02result in /data_prelim

#######################################################

def nnfittingtest02result(x1): # input all 24 electrodes, and Pac1 in column array
    # ---------------------- Input Layer ----------------------------------------------------
    x1_step1_xoffset = np.matrix( \
    [\
[ -84.88053612631119904108],\
[ -83.47581851125410423720],\
[-121.18659594617300001573],\
[-139.77115764554901033989],\
[ -95.43701267011600464230],\
[-169.85549092444699681437],\
[ -81.26321956691209891233],\
[ -68.19124493207979753606],\
[ -74.89963786830500680480],\
[ -66.68987949310890428478],\
[-248.31614584136900703015],\
[-144.81351947451298656233],\
[-381.67581736986898022224],\
[-263.65967161959298437068],\
[-316.01024257373097725576],\
[-407.85370753967998780354],\
[-248.60996881995998819548],\
[-139.84086992320101217047],\
[-145.71517722472898981323],\
[-224.61339996834900034628],\
[ -78.12736264798969898493],\
[ -86.84011979990650331729],\
[ -78.83487347892059915466],\
[-104.04121313340600352149],\
[ -39.66735782428180101533],\
[ -34.10132096167860282776]\
]    )     # size(x1_step1_xoffset) = 25x1

    x1_step1_gain = np.matrix( \
    [\
[   0.00573964785345471972],\
[   0.01099225484586929986],\
[   0.00290217806422432990],\
[   0.00427865806464344973],\
[   0.00625058931337648012],\
[   0.00258651887883934017],\
[   0.00683070745549746016],\
[   0.01998909509282180072],\
[   0.01560421220770739914],\
[   0.00862016632337253043],\
[   0.00608873054902561977],\
[   0.01017025648089880073],\
[   0.00321523691736586015],\
[   0.00455252911677989959],\
[   0.00309130489393961982],\
[   0.00330557799065236014],\
[   0.00485138347860428025],\
[   0.01141998996818270086],\
[   0.01099681722604430034],\
[   0.00667679451747589959],\
[   0.01847813363637559880],\
[   0.01293224494508599977],\
[   0.01774828576637330069],\
[   0.01318114832480410045],\
[   0.01846666666666670065],\
[   0.02148080642716389960]\
])     # size(x1_step1_gain) = 25x1

    x1_step1_ymin = -1;


    # -----------------------Layer 1 --------------------------------------------------------

    b1 = np.matrix( \
    [\
[  69.43505346567107494593],\
[  -6.67765957477449134672],\
[ -21.23273596042815469787],\
[  15.42986340344786277967],\
[   3.13977942094526962791],\
[ -22.88451217825634032010],\
[  20.47282024353307150477],\
[   3.87265005875849332995],\
[ -20.61059121511219061063],\
[  15.68154501523377319927]\
])     # size(b1) = 10x1

    # size(IW1_1) = 10x25
    IW1_1 = np.matrix( \
    [\
[  14.14346739942132558099,   0.79267543316724087266,  -9.16231876874064887772,  -5.23519283118860379034,   2.39996522820838587009,  -1.23585719656198200767,   1.58061446651591386292,  -1.44740275925876060725,  -0.06389935269658386652,   2.90242347616220675022,   0.56741882361840323412,  -0.84653410604504086834,  -9.58466286145282175823, -11.83906199562886385479, -10.43229943838659146138,   9.63265816891986048631,  -3.45839906968424104505,   0.94167275639408476451,   0.44674800866318697956,  -9.99226805268781070879,   0.87142835989801625818,   1.42928126045266656874,   1.32697601828295308479,   0.89116277706626934041, 241.12636060160028250721,  -1.86464490569752250337],\
[   1.50851757175561473900,   0.32196345108024382320,  -4.71325392278048127537,   0.74452659219136962943,  -0.01551987466811374783,  -5.72330308798250264601,   0.27850705852868901102,   0.77682739079157636652,   0.79287837116699932949,   1.62818961237734050052,   4.46409877014771616643,   1.88700989600375113042,   3.53367092249997361009,   0.53515616302240576196,   2.65452719460735497847,  -1.08194529506771353056,  -0.47367379931956277161,   1.99797802896068255052,   2.65606717920341806050,   2.28193719213967805004,   1.13761551125443260979,   1.15268778521139125814,   0.94036159191604207219,   0.89758202413913201756, 377.40650174536904160050,-354.80386396056019293610],\
[   0.42921074745997084277,   0.22320180477843482025,   0.96795001662750690397,   0.59598160723303050812,   0.43109590192727109681,   1.05872500565215443835,   0.39430710639454114030,   0.11901894234227257396,   0.13781615251949291823,   0.34145186206091127090,   0.40554694444964090039,   0.26131183261849133537,   0.92444103406748878715,   0.61777906014667238743,   0.80254242054182434174,   0.80847501098731178182,   0.54861040642315983540,   0.24122287327225727660,   0.22645179587439565050,   0.27583582606914258673,   0.10731890565685466010,   0.21831598032736682091,   0.15597598566313858326,   0.17366265237319519765,-850.51121337074573602877, 769.23315738450048684172],\
[  -1.42528973686233850771,  -0.57173435024950336469,   3.07918211287695831047,  -0.27004412698831248285,  -0.83076265756112077643,   3.75452571224708808728,  -0.13712785746869360359,  -0.35261353177079735044,  -0.72059543647088120100,  -0.99672509408055398605,  -1.99083183154833198714,  -1.20623910489193075790,  -2.26992954923036105441,  -0.27801477898387216836,  -2.29314030852876493682,  -2.44304507121193470454,  -1.34703313832989479693,  -0.89598492330296974906,  -1.08875492454380551344,  -2.57698887449271563099,  -0.87055964372138694429,  -0.37401280929573998968,  -0.37862656358512980503,  -0.52760456188690274537, 510.30881188612443111197,-485.58378108822779495313],\
[   3.19629224995593386183,  -1.24375080813192884222,  -9.83945288075941526529,  -1.74838686187522784898,  -4.02254867061370902093,  -4.27775206825017040302,  -0.93605148011838257283,   0.35763364870033753640,   0.19941248326650506950,  -0.38528958402181989751,  10.53670437556679218005,   0.65532943840535617941,  -0.97492593571746111181,  -4.54616279596517003370,   2.41765390053325779363,  10.08995453087892535393,   0.19244656539975824039,   0.42859252271981318483,   2.13572383233190032925,   2.62413041208744912680,   1.53929915268948280804,   0.94817471551078658099,  -1.15553024709977170126,   2.05615038935598848369, -43.12312315232256310082, 131.22453693658525253340],\
[   0.58006098374998160683,   0.31783550426055834359,   1.40531310946874454793,   0.86881236491586877335,   0.55797811059919155685,   1.51965442954890361804,   0.53520127135761030868,   0.17298972324483827268,   0.20163523987258791248,   0.45801607254608411868,   0.62423359992204685565,   0.35350055642765582098,   1.17750502564409131878,   0.87751053242434207124,   1.15147793429207401239,   1.19730604491049064109,   0.77576084799728839414,   0.32541797874556943615,   0.32436219692406276227,   0.43394514399467631893,   0.16419972955921491309,   0.30809025463407163636,   0.21911069422106568827,   0.26837106391232612435,-1017.18256662826172487257, 929.78955331260465300147],\
[  -0.51367353107704327719,  -0.25374975637402469308,  -1.02745406704329145242,  -0.63928151654350173771,  -0.52086195940253698922,  -1.17478662624116503288,  -0.47008284054628440840,  -0.13621210212352391489,  -0.15921097954070198566,  -0.39588649678247322949,  -0.42618207676180974719,  -0.30080880000465648472,  -1.09841134617399349693,  -0.67762258627171356729,  -0.90852102505402043686,  -0.89108140509664335305,  -0.61863820352119358592,  -0.28052548840676844177,  -0.26040007107735863023,  -0.30578100778037542451,  -0.12181164492581619818,  -0.24104947213371349934,  -0.17675093389891055606,  -0.18830266581744758980, 734.22330761226885442738,-655.05856455090304280020],\
[  -1.42027776753088819106,   0.27211456065949629624,   3.84096409867193466781,  -1.00448473087521894520,   0.38604373377319178839,   3.31721455753757110685,  -0.62702169265892737648,  -0.57128021240620074916,  -0.69177834711742047791,  -1.36365713706483848711,  -4.61034266122455882453,  -1.43024062341780422436,  -2.24767803245665653833,   0.41382561729126299266,  -1.94512442965313092635,   0.76319977688837048468,   0.24281212885385539124,  -1.62750457329463338318,  -2.22605535289273426258,  -1.64037766807151030868,  -0.80146701507159989752,  -1.09638472315467372376,  -0.20794398152070930896,  -0.30962578653819583518,  40.12351627155172906214, -59.86299941187996864755],\
[   1.22512345452954618530,   0.79877557805741550290,  -2.08911464005692915080,   2.64631676618455458083,   0.54296451375973109332,  -2.58089562802242955541,   1.52998654950246160134,   1.18775509723604044687,   1.35889827410615016134,   1.67205806376840593330,   4.74475117845932281568,   2.18530158618447423180,  -2.20670845018175043606,  -0.60725835450409582084,   4.55584403397130621727,  -1.47523131186338929410,   0.13048903248284260403,   2.31194897347149197486,   3.35685686152675799576,   3.74243401505743245039,   0.97930673428189007890,   1.83062582919515359769,   0.93391355062412206856,   0.78045573928091394045,-123.76915609608805368680,  73.73255362568845328042],\
[  -1.48757200298604019828,  -0.60611001197830938203,   2.88108561381953354186,  -0.44022493097895809555,  -0.91144692315499920010,   3.44292636394763729157,  -0.30173748523283039358,  -0.40371088312931191489,  -0.77490491494290114627,  -1.10300733075423584850,  -2.18805741968983813450,  -1.28102073187679543764,  -2.37432460373845977131,  -0.41075353942151104913,  -2.54521621091868910369,  -2.54789479242226013511,  -1.45407476162529736285,  -0.99280789485471176992,  -1.21510104584029887675,  -2.63574829583648773124,  -0.89882862820354902933,  -0.46581119440729862324,  -0.41768739835866869337,  -0.56148259081266416182, 539.74187941546404090332,-513.49883846796842590265]\
])

        # ---------------------- Layer 2 ---------------------------------------------------------
    b2 = 1.070708971089904

    LW2_1 = np.matrix( \
    [\
[  -0.29609360215093044610,  -1.09647576361636378550,-140.16080398426970532455, -15.70133829288998938978,  -0.42320755094552958697,  67.13612339394684624949, -72.62656643891311603056,  -1.40246111408477691640,   0.77125681401242229818,  16.41961943843661231313]\
])
        # size(LW2_1) = 1x10

        # ----------------------------- Output 1 -------------------------------------------------
    y1_step1_ymin = -1;
    y1_step1_gain = 0.005555555555556;
    y1_step1_xoffset = -180;

    # ===== SIMULATION ========

     # Dimensions
    Q = x1.shape[1]

     # Input 1
    xp1 = mapminmax_apply(x1,x1_step1_gain,x1_step1_xoffset,x1_step1_ymin);
    # size = 25x1

     # Layer 1
    a1 = tansig_apply(npm.repmat(b1,1,Q) + np.dot(IW1_1,xp1)) # sending a 10xQ to function
    # size = 10xQ(1)

     # Layer 2
    a2 = npm.repmat(b2,1,Q) + np.dot(LW2_1,a1)

     # Output 1
    y1 = mapminmax_reverse(a2,y1_step1_gain,y1_step1_xoffset,y1_step1_ymin);

    return y1
# end

# ---------- MODULE FUNCTIONS ---------------------

 # Map Minimum and Maximum Input Processing Function
def mapminmax_apply(x,settings_gain,settings_xoffset,settings_ymin):
    y = x - settings_xoffset
    y = np.multiply(y,settings_gain)
    y = y + settings_ymin
    return(y)

# Sigmoid Symmetric Transfer Function
def tansig_apply(n):
    a = 2 / (1 + np.exp(-2*n)) -1
    return(a)

# Map Minimum and Maximum Output Reverse-Processing Function
def mapminmax_reverse(y,settings_gain,settings_xoffset,settings_ymin):
    x = y - settings_ymin
    x = x / settings_gain
    x = x + settings_xoffset
    return(x)

#---------------------------------------------------------------------------------

def listen():
    rospy.Subscriber("/rh/tactile/", BiotacAll, callback4)
    rospy.spin()

# hand_commander.move_to_joint_value_target_unsafe(start, 3, True)
# hand_commander.move_to_joint_value_target_unsafe(close, 3, True)
# time.sleep(2)

if __name__ == '__main__':
    hand_commander.move_to_joint_value_target_unsafe(start, 1, True)
    hand_commander.move_to_joint_value_target_unsafe(close, 2, True)
    slip = Slip()
    time.sleep(1)

    listen()