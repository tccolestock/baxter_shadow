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

# Uses normalized values


rospy.init_node("sr_grasp", anonymous=True)

# arm_commander = SrArmCommander()
hand_commander = SrHandCommander()
time.sleep(1)

class Slip(object):
    """docstring for Slip"""
    def __init__(self):
        self.last3 = [0]*5
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

# Down
    # realtime =[3594,3720,3671,3720,3498,3653,3740,3540,3560,3481,3213,3529,2773,2991,2559,2746, 3129,3359,2892,2629,3658,3665,3522,3368]
    # realtime.append(1970)
    # realtime.append(2293)

    # Up
    # realtime = [2899,3309,2453,2744,2762,2415,2977,3266,3214,2885,3582,3681,3711, 3533,3056,3723, 3584 ,3470,3048,2947,3522,3338,3479,3154]
    # realtime.append(1934)
    # realtime.append(2339)

    # Grasp
    # realtime = [3549,3701,3632,3684,3476,3607,3710,3531,3547,3451,3247, 3543,3016,3156,2703,2980,3244,3385,2930,2706,3650,3645,3529,3368]
    # realtime.append(2056)
    # realtime.append(2325)




    if slip.flag == 0:
        slip.rebase(realtime)
    else:
        features = np.array([0]*26)
        # features[:24] = np.array(realtime[:24])/np.array(slip.base[:24])
        s = sum(realtime[:24])
        s = s/(100*1000) # *100 to make it a %, *1000 to scale it to same mag as pressure
        # print(s)
        features[:24] = np.array(realtime[:24])/s
        features[24:] = realtime[24:]
        # features.extend(slip.base)
        # features.extend(realtime)
        # print(features)
        features = np.matrix(features) # convert list to numpy matrix
        # print(features)
        angle = nnfittingtest02result(features.T) # transpose matrix to create column vector(s)
        slip.new(angle)
        # print("slip: %f ------ last3: %s" %(slip.mvavg, slip.last3))
        print("slip: %f" % slip.mvavg)
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
[3765.39023372287010715809],\
[4297.52712854758010507794],\
[3196.00792161768004007172],\
[3569.75105683418996704859],\
[3584.05636375496987966471],\
[3139.18897761077005270636],\
[3866.03320455257016874384],\
[4209.45798999588987499010],\
[4172.75764853294003842166],\
[3747.25905816017984761856],\
[3879.18421353961002751021],\
[4355.25403585223011759808],\
[3197.43390654634004022228],\
[3518.19558825401009016787],\
[2891.73427991886001109378],\
[3227.07845731149018320139],\
[3764.99319502918979196693],\
[4133.14324676130036095856],\
[3578.46233882056003494654],\
[3222.91287649080004484858],\
[4410.15526344273985159816],\
[4317.11436284964020160260],\
[4329.28001538017997518182],\
[4058.60681564787000752403],\
[1225.00000000000000000000],\
[2149.00000000000000000000]\
])     # size(x1_step1_xoffset) = 25x1

    x1_step1_gain = np.matrix( \
    [\
[   0.00247521855046878986],\
[   0.00466084262162979983],\
[   0.00135029571927940996],\
[   0.00172707112705488999],\
[   0.00226976135537762004],\
[   0.00132083751299503995],\
[   0.00223594229103833003],\
[   0.00699765214202364982],\
[   0.00563910423081966008],\
[   0.00294411724141841000],\
[   0.00248783850820105982],\
[   0.00452231556741184999],\
[   0.00120855396152154989],\
[   0.00181861408304252996],\
[   0.00149479794736294001],\
[   0.00122139190225452007],\
[   0.00219512240840361982],\
[   0.00508483513322679007],\
[   0.00500412431808121023],\
[   0.00322069375754667009],\
[   0.00907388234344221924],\
[   0.00600470721686368995],\
[   0.00905842375112637932],\
[   0.00762383339135397006],\
[   0.00090950432014552099],\
[   0.00781250000000000000]\
])     # size(x1_step1_gain) = 25x1

    x1_step1_ymin = -1;


    # -----------------------Layer 1 --------------------------------------------------------

    b1 = np.matrix( \
    [\
[   0.97074850939294288210],\
[  -1.96405600963920035085],\
[   2.40960393551555940306],\
[   0.16453812068544035330],\
[  -1.09058755782512362131],\
[   1.51848321476256908902],\
[  -1.05601815743440297091],\
[   0.01289460926186100469],\
[   5.66956997432559717254],\
[   0.97461368446252560904]\
])     # size(b1) = 10x1

    # size(IW1_1) = 10x25
    IW1_1 = np.matrix( \
    [\
[   0.33935639295332248322,   1.01748829918574124065,   0.43851159968118913479,  -0.00379495592007590655,   0.65589049841126290108,   0.24970394599431930405,   0.20559442346539363866,   0.50004723607759937742,   0.35677561730889478797,  -0.53222449920246583943,   0.27726675133878592172,   0.40233405999423776933,  -0.19516215843239720362,  -0.43546723464298381367,  -0.98283264478069976011,  -0.86621048085670693251,   0.12694779035075987039,  -0.13243431537454861124,   0.05302245238606935474,   0.24597846213042987018,   0.65055858435664548622,   0.58518354055699162153,   0.68926548980918400655,   0.05765766101761512358,   0.34867919327604013358,   0.04446179313110660419],\
[   0.56344452456207405611,   0.36128527445659658346,   0.43927940929152736738,  -0.60088271794375280876,  -0.75156618202432534659,   0.70820591253967080103,   0.12741524771034690167,  -1.15786037589984025509,  -0.72476725748511638958,  -0.47307927510515113134,   0.55121106294159827321,   0.46064699310813284061,   1.61891853659365692053,   0.03327193789244893563,  -0.05441993342525423133,   0.89552288098634413860,  -0.72442536839972448970,  -1.29322615772555749025,  -0.94611399560524878094,  -1.89214856864564584527,  -0.65891208445803317595,  -0.16661443406107687148,  -0.36646930764002383363,  -1.44611368000055162675,   0.63386412339576725383,   0.05407880725844919217],\
[  -0.18167429684545649549,  -0.46741737787162368489,  -0.40986116046434029059,  -0.03903506487582787676,   0.20945279128874874641,  -0.07838573219595044994,   0.06900280582953234598,  -0.34641633809551952705,  -0.45833738673682966613,   0.14345586947231556141,   1.21449058337327686630,  -0.88591988703357471557,   0.48719558501497284286,  -0.27983359977280741759,   0.86888115618583139366,  -0.13120461201045882649,  -0.90056005036056352431,  -0.48533335680315142779,  -0.87515572384017581431,  -1.99405534477097012847,   1.03186624673233806604,  -0.61462025510783002691,   0.83029366016803385353,   0.30883633420789058377,  -1.54988727488582034297,  -0.28505016812848610330],\
[  -0.89763931234161198347,  -0.44610256961246014251,  -0.12971714183230717565,  -0.06611195605278806509,  -0.54660724701280616333,   0.69071090290259895550,  -0.30974669926568076939,   1.36278944977614391298,   1.20569720625612775144,   2.21589520630784209132,  -3.43401194993489555785,  -0.34732637515896153113,  -1.06986348337081804694,   0.55175714068744163843,  -0.90669361860653729934,   0.02638298195871686611,   1.25875095662493063919,   0.34918340461634411742,   1.34619707772289087799,   1.56571923412818869714,  -1.12599925873983708335,   0.27485750834332278725,  -0.60557954989068096818,   0.62777235991039059826,   0.31925864756169342051,   0.93065452890697064614],\
[   0.26137473237474057042,  -0.68673662366491639020,   0.53984445772678602005,   0.05820890481658239773,  -0.16141960845231498323,   0.73549694195392156804,  -0.37706804271702110132,   0.41407341539508124573,   1.04666742412569835352,   0.92132215344489210018,  -1.99028292886336388712,  -1.07106000888480701327,  -1.12856092469655933108,  -1.57142868884194930601,   0.03832859658522166230,   0.61577517712010021445,   0.54857847934780901156,   1.25821702981936178745,   1.16370187337308084174,   3.67488951373264516320,  -1.83149294438251319939,  -0.25713567102809353759,  -1.62731022406308856176,   0.78439009324689379365,   0.25398343547836860212,  -0.06076618658811610424],\
[   0.95022353317143160023,   2.85134214566250898670,   0.88365540268847442285,   1.01755089442124080357,   1.78391129702966533266,   1.60365352301869146245,   2.56967969720115396726,   2.22278448378748860748,   2.34302984542164649540,   0.47295472404921828913,  -3.41338635181241478733,  -0.45299819636820809476,  -3.11428146191450760938,  -0.50816276262823389942,   0.98992824860635109108,  -4.43703395349803120240,  -1.89248295437984070944,  -0.14104259790863310497,  -1.90585719947562126642,  -3.04094316170412160005,   5.00254194208318914150,   3.34120518258603027562,   4.81322123803709445156,   4.05129045633974183005,   0.33526547158996850007,  -2.95054457847470708032],\
[  -1.05184133151079373114,   0.88650860410335141637,  -1.14161604972935659674,  -0.48254640455400310550,   1.14478264195798140612,  -0.74782073751115529792,   0.54342508794753108248,   0.74319348266718854301,  -0.05083919510756603616,  -2.50362156844056071847,   1.82220720786164314120,   1.40295827570385034377,   2.48412468676212583318,  -2.25485775099949670164,  -2.04388456761042425214,   2.12486131365692276773,   0.10607249731285800509,   0.50607539046204408351,  -0.47578653552107741387,  -3.09705931186675620381,   1.55510590187638220172,   0.17041499995366760078,  -0.21183444707375351235,  -0.20557009566619613428,   0.26271192615652877445,  -1.02794381687485270049],\
[  -0.00884801583101149126,  -0.85948501465585636083,   0.80236801366871801644,  -0.90404001815084289451,   0.31915261744327722004,   0.34814627351146820722,   0.52887882104849914811,   0.31077950376491714213,   1.22688284864160923604,   1.81508208033621398769,  -0.42938154478885592891,  -1.66389212328411817410,  -0.53488668286216267678,   0.13938180078606546286,   0.12636448555085588730,  -0.23445314597835881232,  -0.09689956607625871410,   0.06569933367933647461,  -0.19965728450001954397,  -0.12694482229626810610,  -1.39231651320461824461,  -0.65688868883115725250,  -0.83356392579838434198,  -0.44579570398443885137,  -1.46047235891110971373,  -1.11714163171234481808],\
[  -1.35662202252982200257,  -3.28706489056554262973,   0.15528649083807966047,   1.25767340030156438146,  -1.61845526145425089304,  -0.68300152727415941456,  -1.96085362219364744973,  -9.38698566553902580267,  -8.20414551601035491046,  -4.87690831958416382008,   2.60588497979832611762,  -0.96976621667430984441,   6.35685769011722179300,   5.82229283233333116954,  -0.72025849555416709968,   6.16285792828130674081,   0.62214323184548381285,  -8.03792921512145142060,  -4.95824433883599624551,  -4.69380488745526847083,  -4.72784450104793485536,  -5.56330451772487855067,  -1.01341811204569420823,  -5.63382945740341867236,  -0.26600451548930048595,   7.23989208690992924744],\
[  -0.23184520563163352191,  -1.35434282406734185500,  -0.36027473715337166027,   0.10068452407285270744,  -0.56181571178889977425,  -0.43669766301117030061,  -0.02154625052567553031,  -0.54763565552022830385,  -0.60339070295416863399,   0.03764043744817567594,   0.08359574008160963743,  -0.66571955172608388018,   0.18673501707117776127,   0.86577200559685818515,   0.38154436582874268202,   0.23128097223584889730,   0.65319407089625991514,   0.32313244282585912304,   0.41741483700195980422,   1.30335765363734656930,  -1.82731376814459123281,  -0.78897753402907022657,  -0.24825400114546805619,   0.75884420418719067403,   0.18766541182178259062,  -0.25435581229231069011]\
])

        # ---------------------- Layer 2 ---------------------------------------------------------
    b2 = -4.579363017158631e-07;

    LW2_1 = np.matrix( \
    [\
[   0.00000155937081138415,  -0.00000022530051382605,   0.00000022373986776002,  -0.00000005848797945103,  -0.00000001113679307347,  -0.50000027297499605172,  -0.00000023615642838354,  -0.00000001717013152365,   0.50000017860729573105,   0.00000126050926055014]\
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
