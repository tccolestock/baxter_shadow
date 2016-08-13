#!/usr/bin/env python

### !!!  UPDATE THE DESCRIPTION !!! ###

# Psuedo code:
# This script will allow the shadow hand & UR10 to pick up a "bottle" and then deliver it to the Baxter robot. The Biotac sensors will serve as the feedback to the SH and to the Baxter.
#
#     - Shadow moves into position. No feedbackself.
#     - Shadow grabs the bottle.
#         - Use Biotac feedback to properly grab the object?
#         - Use BT feedback to ensure the object is grabbed.
#             -   NN classification of "neutral" or "grabbed".
#     - Shadow moves to the release position.
#         - The BT proof that the object is still in its hand triggers Baxter to start.
#     - Baxter will reach for the object (Different Script)
#         - Baxter uses its gripper feedback to ensure the object is grasped.
#     - Baxter pulls up on the object, causing a shadow classification that slip is occuring.
#         - If the slip is upward, shadow will release the object.
#             - Using a moving average, normalized with reference. Based on grasp_4_norm.
#             - Angular fitted classification.
#     - After Baxter has the object shadow returns to the start position.
#     - Baxter places the object in a new location outside of shadow's reach.
#


from __future__ import division
import rospy
from std_msgs.msg import String, Float32, UInt8
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
from sr_robot_msgs.msg import BiotacAll
import time
import numpy as np # for: exp(), .shape, array, matrix
import numpy.matlib as npm # for: npm.repmat()
import zmq
import msgpack


rospy.init_node("move_test", anonymous=True)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5556")


hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

time.sleep(1)

# =============== Define Slip Class ===============
class Slip(object):
    """docstring for Slip"""
    def __init__(self):
        self.last3 = [0]*5
        self.mvavg = 0
        self.base = [] #np.matrix( np.zeros((26,1)) )
        self.flag = 0
        self.grip_db = []
        self.grip_mat = np.matrix([[0]*10]*24)
        self.grip_avg = np.matrix([[0]]*24)
        self.grip_count = 0
        self.pac_db = []
        self.pac_mat = np.array([0])
        self.pac_avg = 0
        self.pdc_db = []
        self.pdc_mat = np.array([0])
        self.pdc_avg = 0
        # self.pdc_count = 0
    def new(self, x):
        self.last3.pop()
        self.last3.insert(0,x)
        self.update()
    def update(self):
        self.mvavg = sum(self.last3)/len(self.last3)
    def rebase(self, x):
        self.base = x
        self.flag = 1
    def gripMean(self, x):
        if len(self.grip_db) < 51:
            print("x:")
            print(x)
            # self.grip_db[0:24,i] = x
            self.grip_db.append(x)
            print("grip_db")
            print(self.grip_db)
            # self.grip_avg = np.average(self.grip_db,1)
            # print("grip_avg")
            # print(self.grip_avg)
            self.grip_mat = np.matrix(self.grip_db)
            self.grip_mat = self.grip_mat.T
            print("grip_mat")
            print(self.grip_mat)
            self.grip_avg = np.average(self.grip_mat,1)
            print("grip_avg")
            print(self.grip_avg)
            self.grip_count += 1
    def pacMean(self, x):
        if len(self.pac_db) < 51:
            self.pac_db.append(x)
            self.pac_mat = np.array(self.pac_db)
            self.pac_avg = np.average(self.pac_mat)
    def pdcMean(self, x):
        if len(self.pdc_db) < 51:
            self.pdc_db.append(x)
            self.pdc_mat = np.array(self.pdc_db)
            self.pdc_avg = np.average(self.pdc_mat)

# =============== Define Listen (Subscriber) Function ===============
def listen():
    rospy.Subscriber("/rh/tactile/", BiotacAll, callback)
    rospy.spin()

def pacZero(pac):
    b = pac-slip.pac_avg
    pac_zerod = np.abs(b)
    return(pac_zerod)

def pdcZero(pdc):
    b = pdc-slip.pdc_avg
    pdc_zerod = np.abs(b)
    return(pdc_zerod)

def pdc_callback(data):
    p = int(data.tactiles[0].pdc)
    slip.pdcMean(p)

# =============== Define Callback Function ===============
def callback(data):
    realtime = list(data.tactiles[0].electrodes) # comes in as a Tuple, convert to list
    # realtime.append(data.tactiles[0].pac1) # append the Pac1 value
    pac = int(data.tactiles[0].pac1) # append the Pac1 value
    # realtime.append(data.tactiles[0].pdc)
    pdc = int(data.tactiles[0].pdc)
    if slip.flag == 0:
        slip.rebase(realtime)
    if slip.grip_count < 50:
        slip.gripMean(realtime)
        slip.pacMean(pac)
        # slip.pdcMean(pdc)
    else:
        # checkGrasp(data) # check that shadow has the object and send boolean to Baxter
        features = np.array([0]*24)
        # s = sum(realtime[:24])
        # s = s/(100*1000) # *100 to make it a %, *1000 to scale it to same mag as pressure
        features[:24] = np.array(realtime[:24])
        # features[24:] = realtime[24:]
        features = np.matrix(features) # convert list to numpy matrix
        # print(features)
        # features = ((features.T - grip_mean)/grip_mean)*100
        features = ((features.T - slip.grip_avg)/slip.grip_avg)*100
        # print(features)
        angle = nnfitting(features) # transpose matrix to create column vector(s)
        pac_zerod = pacZero(pac)
        pdc_zerod = pdcZero(pdc)
        slip.new(angle)
        # print("slip: %f" % slip.mvavg)
        print("slip: %+6.2f \t\t pac: %6.2f \t\t pdc: %6.2f" % (angle, pac_zerod, pdc_zerod))

        if (pac_zerod > 50) and (pdc_zerod > 50):

            if (slip.mvavg > 170) and (slip.mvavg < 190):
                print("Upward slip detected!")
                # Release the object
                joint_goals = hand_start
                hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2, True)
                # Move back to start
                time.sleep(5)
                joint_goals = arm_start
                arm_commander.move_to_joint_value_target_unsafe(joint_goals,5,True)

                time.sleep(1)
                rospy.signal_shutdown("Slip was Detected")


def checkGrasp(bios):
    first_pdc = bios.tactiles[0].pdc
    thumb_pdc = bios.tactiles[4].pdc
    if (first_pdc > 2300) and (thumb_pdc > 1700):
        msg = 1
    else:
        msg = 0
    print(msg)
    packed = msgpack.dumps(msg)
    socket.send(packed)

def grip_basing():
    for i in range(10):
        rospy.Subscriber("rh/tactile", BiotacAll, grip_basing2, i)

# def grip_basing2(data, i):
#     print("basing # %s" %i)
#     e = data.tactiles[0].electrodes
#     e2 = np.matrix(e)
#     slip.gripMean(e2.T, i)

def grip_basing2(data, i):
    print("basing # %s" %i)
    e = data.tactiles[0].electrodes
    # e2 = np.matrix(e)
    slip.gripMean(e, i)

# =============== Define NN Fitting Function ===============
def nnfitting(x1): # input all 24 electrodes, and Pac1 in column array

    # --------------- Input Layer ---------------
    x1_step1_xoffset = np.matrix( \
    [\
[ -18.59732414793739962988],\
[ -10.88783839408080034161],\
[ -32.11429457375790263995],\
[ -25.55865022451209966903],\
[ -20.85104764553720002596],\
[ -32.75991395355679713930],\
[ -19.93117288667309949801],\
[  -7.25246970902099974410],\
[  -9.31558083318473073575],\
[ -15.85507964507709921520],\
[  -7.78055098171823011910],\
[  -4.07265672183249005656],\
[ -19.19346153293519918748],\
[ -13.43005530579170070382],\
[ -14.00349364734269919097],\
[ -16.00954518247259983355],\
[  -8.86815479770167058859],\
[  -2.22646399752560020247],\
[  -3.08861322068840982169],\
[  -5.25203258105115988741],\
[  -3.82000883017667014485],\
[  -8.78292236449726004821],\
[  -2.47499178133658981338],\
[  -6.59452535192612998571]\
])     # size(x1_step1_xoffset) = 26x1

    x1_step1_gain = np.matrix( \
    [\
[   0.09820880886426590350],\
[   0.16845500379650699130],\
[   0.05835142703284870030],\
[   0.07425003367003370214],\
[   0.08899157264957270608],\
[   0.05675214380121600122],\
[   0.09401060567556120129],\
[   0.23110276315789499146],\
[   0.18554803508771899878],\
[   0.10780318062138000146],\
[   0.12133005504587200674],\
[   0.26322004901960799339],\
[   0.05146786975717439899],\
[   0.08032186163522009315],\
[   0.05753651482284889707],\
[   0.05068089485458610044],\
[   0.10278611814346000353],\
[   0.30233680119581501922],\
[   0.24402225941422600597],\
[   0.12630330188679200765],\
[   0.41541431818181800972],\
[   0.20841961904761899249],\
[   0.45208067940551999175],\
[   0.24888511685116901373]\
])     # size(x1_step1_gain) = 26x1

    x1_step1_ymin = -1;

    # --------------- Layer 1 ---------------
    b1 = np.matrix( \
    [\
[   1.31697971765866084226],\
[  -9.39081917678928057569],\
[  -0.54749458018119645519],\
[  -0.57224666771926613329],\
[  -0.16545171646601811166],\
[  -0.57590870463824883618],\
[   0.57614647420275344469],\
[  -1.99164482122249131280],\
[  -1.76472063620873553802],\
[  -3.90130145316825105439]\
] ) # size(b1) = 10x1

    IW1_1 = np.matrix( \
    [\
[  -0.56942165049974935442,  -0.53444647055457727980,   0.28892098507264096785,  -0.61323735354452257873,   0.13081367514477179603,  -0.22080154268755541880,  -0.16712116465008289290,  -0.70125439269926836960,  -0.78634688670593821946,  -0.04536825642263501990,   0.52938363418858391807,   0.14033751266891225651,  -0.02011155574725594433,   0.10794314008903370394,  -0.28199224715462706259,   0.43046048957832816484,   0.33398392872424231825,  -0.34931318582767867387,   0.35785295062642680231,   0.26095218267745573515,  -0.38851300894580881318,  -0.22419921634354270101,   0.44473807323344999309,  -0.28214419436411330988],\
[  -0.65634032092345651055,  -0.90463652203158484433,  -2.65109188979186205515,  -3.72211168516185786359,  -1.90531764042129592163,  -1.74082507848804235451,  -2.51407701615162171294,   2.27016480648099427953,   0.90011015617510092834,   1.09518779732781990077,  -7.85966366287118312073,  -4.24229392970649943351, -14.73212253937146876126,  -9.42018165671247587056,  -1.20173838725667003757, -12.36996050510899536334,  -6.14186500397824097774,   3.54269513688441861632,  -0.17984875247598136605,  -2.43003796242663705485,   2.25443955210351365537,   0.63604079132349766734,   1.20675775584198374801,   2.39946526471296106564],\
[   1.18033752164906857729,   1.12784012266605992991,   1.17437441308987411404,   0.89957014355923214932,   0.82355348721212584540,   1.58877912722034175808,   0.67968902376586948222,   0.25382241877183536749,   0.28161944514273290485,   0.96449372187365212117,  -1.61904153001046080540,  -1.64049340212728589350,  -2.18538263339829041954,  -2.73170201265291678894,  -3.65349428385385976981,  -3.36840034591214454451,  -3.66039971015673470944,  -3.33015100437878253103,  -4.21354793491642976022,  -4.21795981044176837571,   0.06852153365329573254,   0.79020379345986979658,  -1.89961276771275300312,  -0.18644988092710373939],\
[   0.38483887542946471072,  -0.56039501822191495339,  -0.21444551734186373393,   0.27693512253837382353,  -0.01570662339488896816,  -0.49286634798536776980,  -0.47569808028071552952,  -0.11746924124895007091,  -0.46265056371669821544,  -0.29096219051547295154,  -0.31062297087201756751,   0.10485438137960079175,   0.02093235078159124893,   0.31401307490409696710,  -0.12756063416751270423,   0.53115842826433845580,   0.14340203570377307862,  -0.40737669527683367798,   0.18207438411852744364,  -0.26410513619060216328,  -0.06211180502483995558,  -0.18630473111963105626,  -0.45763919267734093355,  -0.17417997244405106216],\
[   0.17839015334847452277,   0.15900431327168679241,   0.18079597447191680937,  -0.50284250205254088506,  -0.44831382543336667501,   0.47473293694429319345,  -0.23721431277670745885,   0.50100829372579214205,   0.39540813902147514680,   0.61577621493550682708,  -0.63844083560507536301,  -0.62689809365375748218,  -1.24373497893523965452,  -0.71552133288784325327,   0.39044726744330487955,  -0.34450831010780974539,  -0.52093831949269386872,   0.38877272434741699803,  -0.03130090195796755848,  -0.16571186813181035258,   0.19609779648770267468,  -0.19026299225962381434,  -0.03373964444905581950,   0.01501983920767984218],\
[   0.10853207292089553371,  -0.05930221284373335400,  -0.40610990396501073230,  -0.45669872290967888917,  -0.20638114548213273847,   0.03900162468693765122,  -0.40115111657907814857,  -0.01330868828871002718,   0.23530548953289665004,   0.26716189001412538051,  -0.77918465363959821257,  -1.31806065636400715491,  -0.97050743433830644058,  -0.75841890510084297716,  -0.58584648589412402497,  -0.74570168446986806465,  -1.04099665370273664067,   0.18979726217837941671,  -0.84103309159656147731,  -0.00157958644928765570,  -0.48395154437305465400,  -0.29039235373003013940,  -0.55032591312777778825,   0.21298957423577188908],\
[   0.47544162368753589698,   0.26527048994729557352,   0.31254304841503549595,   0.12831882050345294610,   0.63445598004955383598,  -0.21303306844723840507,  -0.29806639372894511153,  -0.02036186575847312094,   0.14601496773648828253,   0.27865047794090924693,  -0.36438663119086239783,  -0.35717110527185114144,   0.09797531036311264707,   0.50433359421565915337,   0.00920245545649091452,   0.66120890806242493820,   0.01835928947771467770,   0.33821275026819230813,  -0.42174201276952627548,   0.24208240155057694776,   0.13037220232369425843,   0.14078724864184136156,   0.16741815633705969812,  -0.00168621875886768740],\
[  -0.46886639489009984683,  -0.24488520182386702695,  -0.97536814345640743262,  -1.04199581048875211664,  -0.89937165662052320769,  -0.70178675009203705937,  -0.57346690717488857381,   0.68336820341727355643,   0.62344845870772969132,   1.15591610216969886693,  -2.27953016314055645353,  -2.12998006500296854782,  -3.09726791395316292110,  -3.54624798110571193277,  -1.52953850995172624394,  -1.66544805780939264572,  -2.45558570219589178407,   1.05511527957637940567,  -0.69363727062234059595,  -0.29028331662971035287,   0.19909905961640794736,  -0.21010410147803823544,  -0.04825855039920928791,   0.76716186376181916273],\
[  -0.13149549593747184018,  -0.12876579204702370829,  -0.62155569777698527201,  -0.56621852796012916986,  -0.45920699189315944411,  -0.16457892329994663894,  -0.34958681700731530917,   0.37799793674052972747,   0.50300144349027586799,   0.73486342722463726673,  -1.33955249490749772612,  -1.63186789099831175420,  -2.02236198798417943223,  -2.16220086814825940280,  -1.10065181419104818339,  -0.88146548086204390415,  -1.67386138224068559133,   0.59867382519738576718,  -0.65725651427745179767,  -0.09414725000252611997,  -0.09103745500845891736,  -0.23876461699813805728,  -0.28333265237511373602,   0.44150650203104646607],\
[  -0.91721213826566572180,  -0.54925785079204081374,  -1.74431846287724745537,  -2.15173609489245443172,  -1.55919613539637635924,  -1.58917544703052904254,  -1.23805308288498805425,   1.33714609446348520194,   0.75577964814372944335,   1.60345575832126741034,  -4.53367250715328751198,  -3.18277005075005137158,  -7.03741321579400302966,  -6.51493271657708650224,  -1.85187310833031948398,  -5.19469217072647015954,  -4.06473373981770613739,   2.13555060386954176366,  -0.67107024014879501639,  -0.98726176882599880891,   0.96406287985674576912,  -0.01974573842238017823,   0.49885789339048114410,   1.48487181121149158791]\
] ) # size(IW1_1) = 10x24

    # --------------- Layer 2 ---------------
    b2 = -1.121185639556207e-08

    LW2_1 = np.matrix( \
    [\
[  -0.00000002394751281700,  -0.50496451549206988751,  -0.49999980057769677355,   0.00000000329699677109,  -0.00000039160223395036,  -0.00000463530699408969,   0.00000000202775646272,  -0.00025976471958492827,   0.00006063055001380886,   0.00516844313566053488]\
] ) # size(LW2_1) = 1x10

    # --------------- Output 1 ---------------
    y1_step1_ymin = -1
    y1_step1_gain = 0.005555555555556
    y1_step1_xoffset = -180

    # --------------- SIMULATION ---------------

    # Dimensions
    Q = x1.shape[1]
    # Input 1
    xp1 = mapminmax_apply(x1,x1_step1_gain,x1_step1_xoffset,x1_step1_ymin) # size = 25x1
    # Layer 1
    a1 = tansig_apply(npm.repmat(b1,1,Q) + np.dot(IW1_1,xp1)) # 10xQ to function size = 10xQ(1)
     # Layer 2
    a2 = npm.repmat(b2,1,Q) + np.dot(LW2_1,a1)
     # Output 1
    y1 = mapminmax_reverse(a2,y1_step1_gain,y1_step1_xoffset,y1_step1_ymin);

    return y1
# end nnfitting function call

# =============== NN MODULE (Support) FUNCTIONS ===============

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


# =============== Define Position Dictionaries ===============

# --------------- Hand Positions ---------------
hand_start = { \
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
        }

hand_close = { \
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
        }

# hand_soft_open = { \
#                 'rh_FFJ1': 0.015299964222028228,    'rh_FFJ2': 1.0363475765685581,      \
#                 'rh_FFJ3': 0.2156981911673815,      'rh_FFJ4': -0.09041898402453244,    \
#                 'rh_THJ4': 1.1566064166609298,      'rh_THJ5': -0.4976068025062665,     \
#                 'rh_THJ1': 0.7331455374652653,      'rh_THJ2': 0.24076301002605377,     \
#                 'rh_THJ3': 0.2482866853523483,      'rh_LFJ2': 0.9579282517503304,      \
#                 'rh_LFJ3': 0.22891073506641474,     'rh_LFJ1': 0.0369458923601228,      \
#                 'rh_LFJ4': -0.010122565656606665,   'rh_LFJ5': 0.03884889211514442,     \
#                 'rh_RFJ4': -0.03515217103578468,    'rh_RFJ1': 0.06709122242188231,     \
#                 'rh_RFJ2': 0.8408973912178247,      'rh_RFJ3': 0.34325412649756837,     \
#                 'rh_MFJ1': 0.014565158522349297,    'rh_MFJ3': 0.4407150002695516,      \
#                 'rh_MFJ2': 0.7245574605990543,      'rh_MFJ4': -0.005447683603367941,   \
#                 'rh_WRJ2': -0.106417846398269,      'rh_WRJ1': -0.07804339747071865     \
#                   }

# wrist joints removed from hand:
# start :: 'rh_WRJ2': -0.08740126759572807,    'rh_WRJ1': -0.009642963029241673    \
# close :: 'rh_WRJ2': -0.103164843927744,      'rh_WRJ1': -0.10998772922135532     \


# --------------- Arm Positions ---------------
arm_start = { \
    'ra_shoulder_pan_joint': -1.6755197683917444, 'ra_elbow_joint': 2.391160726547241, \
    'ra_wrist_1_joint': 2.303798198699951,  'ra_shoulder_lift_joint': -1.5533440748797815,\
    'ra_wrist_3_joint': -3.10664946237673,  'rh_WRJ2': -0.08740126759572807, \
    'rh_WRJ1': -0.009642963029241673,         'ra_wrist_2_joint': -1.5882452170001429 \
            }

arm_pickup = { \
    'ra_shoulder_pan_joint': -0.575897518788473, 'ra_elbow_joint': 2.86228346824646, \
    'ra_wrist_1_joint': 1.6754974126815796,  'ra_shoulder_lift_joint': -1.2914817968951624, \
    'ra_wrist_3_joint': -1.5357773939715784, 'rh_WRJ2': 0.05646164732737393, \
    'rh_WRJ1': -0.10736475895393359,         'ra_wrist_2_joint': -1.5881970564471644 \
            }

arm_exit_pickup = { \
    'ra_shoulder_pan_joint': -0.575909439717428, 'ra_elbow_joint': 2.7576346397399902, \
    'ra_wrist_1_joint': 1.8324915170669556,  'ra_shoulder_lift_joint': -1.4485862890826624, \
    'ra_wrist_3_joint': -1.5358369986163538, 'rh_WRJ2': -0.008102103551746979, \
    'rh_WRJ1': -0.10673035727744258,         'ra_wrist_2_joint': -1.5882094542132776 \
                }

arm_midway = { \
    'ra_shoulder_pan_joint': -1.780236546193258, 'ra_elbow_joint': 2.7576465606689453, \
    'ra_wrist_1_joint': 1.8324674367904663,  'ra_shoulder_lift_joint': -1.4485982100116175, \
    'ra_wrist_3_joint': -1.5358369986163538, 'rh_WRJ2': -0.008395394812687014, \
    'rh_WRJ1': -0.10545759885212826,         'ra_wrist_2_joint': -1.5882094542132776 \
            }

arm_release = { \
    'ra_shoulder_pan_joint': -3.027827803288595, 'ra_elbow_joint': 2.6113691329956055, \
    'ra_wrist_1_joint': 1.8882097005844116,  'ra_shoulder_lift_joint': -1.3068426291095179, \
    'ra_wrist_3_joint': -1.4986370245562952, 'rh_WRJ2': -0.103164843927744, \
    'rh_WRJ1': -0.10998772922135532,         'ra_wrist_2_joint': -1.595231835042135 \
            }

grip_mean = np.matrix( \
        [\
            [3545.33800000000019281288],\
            [3697.58733333333339032833],\
            [3611.95333333333337577642],\
            [3675.37666666666655146400],\
            [3470.67133333333322298131],\
            [3578.22266666666655510198],\
            [3699.31733333333340851823],\
            [3512.76200000000017098500],\
            [3525.41266666666660967167],\
            [3411.97066666666660239571],\
            [3306.24400000000014188117],\
            [3579.79266666666671881103],\
            [3108.65933333333350674366],\
            [3192.79399999999986903276],\
            [2652.43333333333339396631],\
            [3020.58133333333353220951],\
            [3248.04133333333311384195],\
            [3371.05533333333323753322],\
            [2916.06599999999980354914],\
            [2677.63000000000010913936],\
            [3655.64600000000018553692],\
            [3647.34333333333324844716],\
            [3548.83333333333348491578],\
            [3372.39333333333343034610]\
        ] )




# =============== Main ===============
if __name__ == '__main__':
    slip = Slip()
    # --------------- Movements ---------------
    # Move arm and hand to start position
    joint_goals = arm_start
    arm_commander.move_to_joint_value_target_unsafe(joint_goals, 8, False)
    joint_goals = hand_start
    hand_commander.move_to_joint_value_target_unsafe(joint_goals,8,False)

    while True: # find pdc baseline
        rospy.Subscriber("rh/tactile", BiotacAll, pdc_callback)
        if len(slip.pdc_db) >= 50:
            break

    # Move arm to pickup location
    joint_goals = arm_pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals,5,True)

    # Grab the object with the hand
    joint_goals = hand_close
    hand_commander.move_to_joint_value_target_unsafe(joint_goals, 4, True)

    ## Check for object here using Biotacs---------------

    # Exit the pickup zone
    joint_goals = arm_exit_pickup
    arm_commander.move_to_joint_value_target_unsafe(joint_goals,5,True)

    # Go to the midway point
    joint_goals = arm_midway
    arm_commander.move_to_joint_value_target_unsafe(joint_goals,5,False)

    # Go to the release area
    joint_goals = arm_release
    arm_commander.move_to_joint_value_target_unsafe(joint_goals,5, True)

    # if len(slip.grip_db) < 11:
    #     grip_basing()

    # for i in range(10):
    #     rospy.Subscriber("rh/tactile", BiotacAll, grip_basing2, i)
    #     time.sleep(1)

    time.sleep(2)
    listen()




# # Release the object
# joint_goals = hand_start
# hand_commander.move_to_joint_value_target_unsafe(joint_goals, 2, True)
#
# # Move back to start
# joint_goals = arm_start
# arm_commander.move_to_joint_value_target_unsafe(joint_goals,5,True)
