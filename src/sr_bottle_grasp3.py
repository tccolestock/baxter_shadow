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


rospy.init_node("sr_grasp", anonymous=True)

# arm_commander = SrArmCommander()
hand_commander = SrHandCommander()
time.sleep(1)

class Slip(object):
    """docstring for Slip"""
    def __init__(self):
        self.last3 = [0,0,0]#,0,0] #,0,0,0,0,0]
        self.mvavg = 0
        self.base = [] #np.matrix( np.zeros((26,1)) )
        self.flag = 0
    def new(self, x):
        self.last3.pop()
        self.last3.insert(0,x)
        self.update()
    def update(self):
        self.mvavg = sum(self.last3)/len(self.last3)
    def rebase(self, x):
        self.base = x
        self.flag = 1


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


def callback(data):
    ffe1 = data.tactiles[0].electrodes[0]
    # ffe2 = data.tactiles[0].electrodes[1]
    ffe3 = data.tactiles[0].electrodes[2]
    ffe4 = data.tactiles[0].electrodes[3]
    if (ffe1 < 3550) and (ffe3 < 3600) and (ffe4 < 3650):
        # hand_commander.move_to_joint_value_target_unsafe(soft_open, 1, True)
        hand_commander.move_to_joint_value_target_unsafe(start, 1, True)
        time.sleep(1)
        rospy.signal_shutdown("Slip was Detected")

def callback2(data):
    features = list(data.tactiles[0].electrodes) # comes in as a Tuple, convert to list
    features.append(data.tactiles[0].pac1) # append the Pac1 value
    features = np.matrix(features) # convert list to numpy matrix
    angle = nnfittingtest02result(features.T) # transpose matrix to create column vector(s)
    print(angle)
    if (angle > 85) and (angle < 95):
        print("slip detected!")
        hand_commander.move_to_joint_value_target_unsafe(start, 1, True)
        time.sleep(1)
        rospy.signal_shutdown("Slip was Detected")

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
    if slip.flag == 0:
        slip.rebase(realtime)
    else:
        features = []
        features.extend(slip.base)
        features.extend(realtime)
        # print(features)
        features = np.matrix(features) # convert list to numpy matrix
        angle = nnfittingtest02result(features.T) # transpose matrix to create column vector(s)
        slip.new(angle)
        print("slip: %f" %slip.mvavg)
        if (slip.mvavg > 170) and (slip.mvavg < 190):
            print("Upward slip detected!")
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
    [3423.00000000000000000000],\
    [3608.00000000000000000000],\
    [3480.00000000000000000000],\
    [3586.00000000000000000000],\
    [3368.00000000000000000000],\
    [3437.00000000000000000000],\
    [3600.00000000000000000000],\
    [3415.00000000000000000000],\
    [3427.00000000000000000000],\
    [3259.00000000000000000000],\
    [3186.00000000000000000000],\
    [3492.00000000000000000000],\
    [2980.00000000000000000000],\
    [3132.00000000000000000000],\
    [2550.00000000000000000000],\
    [2919.00000000000000000000],\
    [3207.00000000000000000000],\
    [3295.00000000000000000000],\
    [2877.00000000000000000000],\
    [2626.00000000000000000000],\
    [3564.00000000000000000000],\
    [3548.00000000000000000000],\
    [3444.00000000000000000000],\
    [3249.00000000000000000000],\
    [1172.00000000000000000000],\
    [2221.00000000000000000000],\
    [2807.00000000000000000000],\
    [3246.00000000000000000000],\
    [2334.00000000000000000000],\
    [2675.00000000000000000000],\
    [2735.00000000000000000000],\
    [2324.00000000000000000000],\
    [2962.00000000000000000000],\
    [3218.00000000000000000000],\
    [3176.00000000000000000000],\
    [2816.00000000000000000000],\
    [2916.00000000000000000000],\
    [3346.00000000000000000000],\
    [2454.00000000000000000000],\
    [2699.00000000000000000000],\
    [2281.00000000000000000000],\
    [2513.00000000000000000000],\
    [2960.00000000000000000000],\
    [3292.00000000000000000000],\
    [2824.00000000000000000000],\
    [2524.00000000000000000000],\
    [3424.00000000000000000000],\
    [3240.00000000000000000000],\
    [3367.00000000000000000000],\
    [3029.00000000000000000000],\
    [1172.00000000000000000000],\
    [2221.00000000000000000000]\
] )     # size(x1_step1_xoffset) = 25x1

    x1_step1_gain = np.matrix( \
    [\
    [   0.01190476190476190063],\
    [   0.01652892561983470079],\
    [   0.01136363636363640038],\
    [   0.01680672268907559830],\
    [   0.01388888888888890026],\
    [   0.01081081081081079927],\
    [   0.01492537313432839956],\
    [   0.01360544217687069922],\
    [   0.01408450704225350063],\
    [   0.00913242009132420041],\
    [   0.00970873786407767080],\
    [   0.01449275362318840077],\
    [   0.00501253132832079999],\
    [   0.00917431192660551030],\
    [   0.00490196078431372976],\
    [   0.00435729847494552985],\
    [   0.00851063829787234057],\
    [   0.01129943502824859967],\
    [   0.01587301587301589989],\
    [   0.01041666666666670078],\
    [   0.01652892561983470079],\
    [   0.01492537313432839956],\
    [   0.01324503311258280053],\
    [   0.01142857142857140081],\
    [   0.00088809946714031997],\
    [   0.01086956521739130058],\
    [   0.00249687890137327995],\
    [   0.00409836065573771034],\
    [   0.00146627565982404996],\
    [   0.00190114068441065006],\
    [   0.00252525252525252980],\
    [   0.00147275405007364006],\
    [   0.00251889168765742980],\
    [   0.00581395348837209034],\
    [   0.00497512437810945004],\
    [   0.00289855072463767989],\
    [   0.00294985250737462993],\
    [   0.00555555555555556010],\
    [   0.00157977883096367005],\
    [   0.00232558139534884021],\
    [   0.00216919739696311981],\
    [   0.00164473684210526003],\
    [   0.00316455696202531988],\
    [   0.00881057268722467042],\
    [   0.00829875518672198997],\
    [   0.00457665903890160011],\
    [   0.00746268656716417983],\
    [   0.00457665903890160011],\
    [   0.00796812749003984050],\
    [   0.00510204081632653021],\
    [   0.00088809946714031997],\
    [   0.01086956521739130058]\
] )     # size(x1_step1_gain) = 25x1

    x1_step1_ymin = -1;


    # -----------------------Layer 1 --------------------------------------------------------

    b1 = np.matrix( \
    [\
    [  -0.00553220201026869036],\
    [  -5.85732815526617933699],\
    [   0.45882579891583691323],\
    [   6.91831147606066565459],\
    [  -0.75989166031963029102],\
    [  -1.04054966966198803036],\
    [   5.39501357223974586930],\
    [   0.74156598839043852411],\
    [   1.84543942110671133072],\
    [  -0.90068866144240655736]\
    ])     # size(b1) = 10x1

    # size(IW1_1) = 10x25
    IW1_1 = np.matrix( \
    [\
        [  -6.30633833033421353775,  -2.68729625932623461537,  -0.71140062185446772958,  -5.24638529293674693577,   5.11174529985295844625,  -0.20050619277721121181,   0.99877352572161903765,   5.02000463026348509032,  -0.59510056858103588251,  -3.50676392593384100849,  -0.24885171782160420140,  -0.05679314637714372171,  -1.11878168528790356007,   0.16232728187878112114,   1.68935760510156440795,  -6.03445118255380563710,   2.06848179236553919580,   2.12055910227713084737,   0.26795218188272945659,   4.82895636018403706657,   3.24982711886527297906,  -1.58424882345127926442,  -0.97288037290976936422,   2.37274073714875610719,  -0.33366060981305784017,  -0.78059838333227815621,  -0.79585077468123588407,  -0.38285461989025332263,  -1.33099935738913077721,  -1.19226512650641480739,  -1.04791926998323048359,   0.00572568967104841123,  -0.28361622700876104597,   1.10378993671004677068,   0.31194067728777197246,   0.52013313278788986249,   2.16242933161983907908,   4.10488287620481440143,  -0.34779669017552145016,   3.09367828918621956547,  -0.35394482709469632375,  -1.93358827965330237753,  -1.62740279940580978568,  -0.84797614176803681740,  -1.64085625201355411740,  -1.24836256702241210625,   0.23218658330186847660,  -1.86112471390447531405,   1.39549442060965933621,  -0.47476643641904658910,   0.01502491247563686114,  -0.23452348178883586360],\
        [   5.20587396124304024880,  -2.70274953425704289600,   0.51019492076998662622,  -4.70923333423067447256,  -2.02920955241653366841,   3.43500864018242246800,   1.35378294218095973633,  -2.50831753902304654957,  -1.33142176175264492777,   3.14735677738086261357,   1.83080238971319486652,   0.97927867495920084551,  -0.94673317472016704777,  -1.54798160192005296310,  -3.71346889894380005614,  -3.68705729125189884954,  -3.34058828848742006556,  -1.54339172299804960886,  -0.77119418782873372376,  -0.16366795124788555138,   1.59266620359483579428,  -1.12222969517545623219,  -0.77066924834359773477,   1.25636261471180121774,   1.27687957617688185863,   2.55456344781813760036,  -1.18953001475541331189,  -0.94347844652745327387,  -2.21760228007019311036,   0.20828244231604955972,   0.34443799812061903776,  -0.73793458987988513886,   0.27884435463721751747,   0.43989361771593471495,   1.17431516861941553920,   4.22581993847856640656,  -3.07625351006756186933,   0.33067682651770158309,  -0.83155353537886633752,  -0.05540689037930953548,  -2.54041612031308083886,  -2.28747635557752193591,   0.45362076935851708415,   1.06041468908884550792,   0.17016755579256331865,   1.61126351185518834619,  -0.69290106630888159689,  -0.76932538214828860568,   2.36515957822252742559,   1.35362806618601450026,   0.97597999005655555749,   2.02485063483167815335],\
        [  -0.73209640248636065341,  -0.19876075966957745811,   0.97707158534059657118,   0.08160752316063252243,   1.28742882405168068694,   1.25560791515074177127,   2.65807228657736560606,   1.63187827170737786986,  -0.45110352945906290190,   1.43426788166386853796,  -0.45454690769952554863,  -0.01144789569752002745,  -1.56689478535302106899,  -1.23489833697460538176,  -0.58447813308087814832,   1.28507730031721667530,  -3.25188890926617890642,   1.74459915073897886550,   0.88458636756270370149,   0.40090507303397715511,   1.43547807957970241510,   2.85847272181289691062,   0.94259533679004292583,  -1.18687762268256924436,   0.61804268394385097896,  -1.02613346794303983422,   1.85711258231343978586,   1.62140395929595015900,   1.55809416203726924799,   1.04804378951707088952,   1.51183744127392905021,   0.88919530416575331966,   1.83764898846249136177,   1.47601148625394662517,   1.27077117531800332095,   0.99082617537264172647,   0.06018785924136915710,   0.59776228519100216285,   0.37961054542349531582,   0.93320678242496857280,   0.78663022419061900958,   0.59772051392065750353,   0.52036829380167781700,   1.82186521012470903358,   0.53804625328819655206,   0.21771431013291905887,   2.17717498864921310542,   1.80883080608687607516,   2.10245542259133344487,   1.19085830058635289141,   0.55627099256472845035,  -0.92617601081284328668],\
        [  10.58088625708781016499,  18.77252252136118571002, -10.18786203582219407338,  10.03848463903147525400,   4.90409832327711558975,  -6.81487655810950787583,   9.28437507098366765490,  -5.63205103934229267537,   7.21041937259102461155, -22.65989740077717939926,   3.53400825963899700710,  -7.65331711153047944407, -25.74470706428089528117,  -1.17064592900343034110,  -7.94672499738939386305, -17.17491674985771510364,  30.33183699183059900406, -17.20283651697585725060,  12.35974468145216143000,  13.79204977314229019214,  14.42670473237665973443, -30.24120366307402463235,   7.36040506479497658177,  -8.06316320759512272787,  -0.09047224572641671125,   0.55048710085060748032,  -1.47251731239271044238,  -0.33240233608933333898,   0.22129229251720292093,   1.11306559567719998149,   0.58609413009713540177,  -1.72188869109411069047,  -0.02925165807546589031,  -0.17549632769559453394,  -0.38639795382825953274,  -1.24711855433377483138,   1.84603245427407025581,   0.58194075765111386023,   5.19925879527299894534,   0.74984908145431650173,   2.25252089947709022155,   5.80514420740813541499,   1.30709927374400325384,   1.55840139420341672327,   1.49002109717208841566,   2.16919082551356012800,  -0.22155888269816359504,   0.28819737263511407965,   0.36672711541000091451,   0.17742172689462648583,   0.01675547652747666136,   0.67623381486826428244],\
        [  -0.85033198632904560821,   2.53065372216626727209,  -1.80528644644396640828,   3.51306772198590921974,   4.20150914379185280723,  -2.45962162764256841996,   2.40383501787175912767,   2.61148853702765082474,  -2.76736640945134082514,  -1.98200657219224907202,   1.48580283218672715151,  -1.55212175867426105391,  -0.40728631735966658312,  -1.30352646443351760652,  -0.95863713702432895580,   0.70286551676095265773,  -1.61290321422349114400,   0.52198667936458242878,   1.95374302839705427104,   4.77801575921249099821,  -3.92180078252660191396,  -0.50338789497394764716,  -0.96107190837330047639,   0.08194606957437836070,   0.41164895298412562674,   3.14260365869081947920,  -1.52910520095742352709,  -0.85329960230293011580,  -0.00081767416840196274,   0.52800377078545368548,   0.37516364692220549593,   0.80624861800498537345,  -0.80256228922997441266,   0.69558501507125503061,  -0.15158818007055704924,  -0.12988195083659134776,   2.19337023541298137275,   0.75536715620021910311,  -1.04385955980644973273,   3.64838916892105213918,  -2.20434066960237506905,  -5.84622967167911333775,  -0.63479963331329891663,  -0.07529062526073666828,  -0.05773679455664284393,   0.85965947036740941734,   0.12208658461510946136,   0.59229735794647841995,   0.46462550989885043062,  -0.31153270723586035107,   0.53969333736994717921,   3.12666063773389391045],\
        [   1.91765818161345436010,  -1.94615779354419782265,  -4.39935546803434807117,   8.44156562080303984885,   2.74975613359388715295,  -5.32336709642004191068,   1.68376845617340165084,   6.74541442040327687835,   2.03233297876370500035,  -1.68415300221554531390,  -0.14264262394687157731,  -5.48003804410902350952,   2.49551642188807676348,  -1.68302306014618174856,   1.36142147572856431204,   2.19471185594601214675,   2.21746558036155683169,  -2.10562309968898375701,  -2.89774861730967714379,  -3.71687159309023540388,  -0.52475966762018066980,  -6.39641359387629560018,   1.10249363901133978771,  -2.92903657032868691701,   0.20845771193147302758,  -0.33239617409812077797,  -0.93257101577357925937,  -0.04054592471067088727,  -0.48219109609499838376,   1.51285148761041132737,   0.49281979024858624250,  -0.28574838360779147850,  -0.31728321948767035376,   0.18083258819235908943,  -0.28929701229602400314,   0.73972702463797568306,  -0.76024053333702212853,  -0.55328798615898244773,  -2.53934407338886058980,   2.53159163763214989373,  -0.81718020950782732648,  -3.38074524836126277805,   0.58228753720927595161,   0.88757379656281354396,   0.11818913233996916623,  -0.51655306716172277870,   0.22699222927529710825,  -0.25872826173815266504,   0.16561142645497686776,   0.35606171810313019810,   0.00831432913853496700,  -0.56954967966292679993],\
        [   7.80326269774864300643,   7.58998023710943847675,  -2.94529470291550232730,  19.76348784956112325517,   6.33684773995984418349,  -9.89818715942789140172,  -2.99996755972052708827,  -3.78039032048342571457,  -6.97172794406699480874,  -1.61807346913533822708, -13.50010411871177140597,   0.26486246999907825384, -22.08105374131638143353,  10.46546194699747722723,   2.90372796618959760906,   5.11163093807337354235,   5.87562471439083200409,   4.65733586268906840644,  16.39095077815927226084,  -2.82277367321309968773,  16.18112961895559465120, -14.48391726628872255844,  -7.61606327452613207640,  -8.54613556734029700124,   0.22685444180808833625,   0.84060187874520042595,  -0.59171919184350219023,  -0.27504551264168281000,   0.19663007236500992803,   1.28875702479681364387,   1.29871137146200865686,  -0.27050238995958519883,  -1.27153600840629543711,   0.27137004970832590001,  -0.35047277197775089652,   0.96298117190937471133,  -1.82243652269365763452,  -0.73056525336519639868,  -5.03814937069991231056,  -0.49671885349232519635,  -0.89723895427877375930,  -7.29859053533594703822,  -0.98022771263384400608,  -0.17774637635846246564,  -0.00644926911883299779,   1.33808744540098545350,   0.06882166426680066951,   0.09649768917275150515,  -0.42660425161867748800,  -0.25158417456941623014,  -0.01842547836312916770,   0.81002337454379669257],\
        [  -3.45690532779606574110,  -0.02976373828063490559,  -5.41259864168272919471,   2.24646569613027491386,   0.44341993176633948126,   2.59635869688751608209,  -4.11105815749977310247,  -3.09045138807890129584,  -1.34946323182735028468,   0.00735734222372758269,   0.02962100215096718017,  -2.30169350971866082389,   1.16508730981411723349,  -2.76466807010675008627,  -0.88511814058265270155,   3.90431247323148200579,   1.01926503583804306530,   0.24657351756433137924,   0.34569463593137284008,   0.29647664762271130900,  -0.65549794956640028420,   2.79978993284379962247,   3.95264402192841490802,   4.34672289618893081808,  -0.64531062676073769335,  -0.64726561717433328536,  -0.32884994379173138057,  -1.58282597762470333969,   2.51881770423277373538,  -0.55825480776267921712,  -1.08321574433822287453,   3.07988472291731030239,   0.76196186188780090642,   1.78924871583159328381,   3.37670283774582236092,   2.74388434883359666472,   0.80057017773967120711,   2.45567805629228619324,   1.09408465637888241417,   2.61598543650705606112,  -3.14320374476852570922,   1.29920798439355578147,   0.83089029887306453670,   2.48366235106618127304,   0.74589975486277781958,   0.22209995332239232391,  -1.33677925448713530443,  -0.89633271095377753390,   1.36774990217970437989,   1.59921683837073524970,  -0.39958687080060634189,  -0.47643171819165602843],\
        [  -1.03565522733894499474,   2.62640039028783300878,  -0.20325601676455143374,  -0.37728239760383064372,  -1.40582877611482892632,   1.76932503693645926113,  -1.60603329659384397310,  -2.34175948398690136898,  -0.87459521674563578397,  -2.85021347529842250523,  -0.10642766318146952564,  -3.47456125162239404247,   1.93323511017825189207,  -2.66531675921946309415,  -0.44324951578764310023,   5.70473451792547336936,   0.51038580956252133092,   3.17396377884799907321,  -1.58725205730960383654,   0.72628987715969672845,   0.72034118251367207364,  -2.68026709164297649579,   4.57205500565489852960,   0.00369544741879274542,  -1.15839325398522530186,  -3.33160960035462805351,  -0.64528820235085759816,   0.08955496104116617007,  -0.00498642069329500830,   1.04554096903624849091,  -0.24830590348736050754,   0.06152008890782077744,   1.55419353024939677077,   1.52447412617802746659,   0.85254686271956092725,   0.17440503994059897153,   1.71241806031516952125,   1.47290318135321074422,   0.75132239881193285669,   1.85475889311772657031,   0.79572256460028667480,   1.10623028314390214888,  -2.14231818075755509057,  -1.50215934291357666730,  -2.45928755561839906818,  -1.85933014152927467144,   1.06010372947828335199,  -1.21519694293393087925,  -0.24854089571616574950,  -0.58606890139179390609,  -0.77525422167886870461,  -3.68312248642455086767],\
        [   0.60549738861929836009,  -2.75007131671172366794,  -2.13523507343601659514,  -1.88620068187479672517,  -2.22726104738285846096,  -2.51370877013274807155,  -2.83627753383609126914,  -2.32217888940084682403,  -3.18227861202132933371,  -1.25669721821549673635,   0.91410347107702760727,   0.78691187375962934336,   1.65298108776235230799,   3.21427368351083631026,  -0.59107101352870028421,   0.81039471270877105891,  -0.09536892289578090265,  -1.90467256841708398873,  -4.21986881663006219156,   1.52575172872071984642,  -0.30613542073909078001,  -3.36960391569019890312,  -1.14071073414593193718,  -2.93752007406649262578,   0.53700197068102772935,   3.23414754817921723529,  -0.21942063985453036379,   0.76042109706640215183,  -0.33341219880792288821,   2.07499770273142569721,   2.21255266065318556556,  -0.59818914654525479069,  -0.10258792805284078220,  -0.19165027091469119536,  -0.96179248613320733607,  -0.05380521491072060014,  -0.62295221220723195898,   1.18472328774450552658,   1.19398207864565408798,   1.18939197717754630013,   3.19306429097857025212,   0.88905181104917185841,   2.01100506074617202401,   2.31482030169165131639,   1.67794531321741557939,   3.42839728244941310820,   0.04777177443349068381,  -0.90184068644664383552,   0.09128497895101479742,   0.39245911288199741840,   0.17423889544705561949,   3.74747707439968280596]\
    ] )

        # ---------------------- Layer 2 ---------------------------------------------------------
    b2 = -2.741158223994022e-09;

    LW2_1 = np.matrix( \
    [\
        [  -0.00000000084659326147,  -0.00000000108308424080,   0.00000000021001268317,   0.50000001062994237078,  -0.00000000218955595851,   0.00000000057543383358,  -0.49999999215590007484,   0.00000000103275746043,   0.00000000108011181442,  -0.00000000079797258034]\
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