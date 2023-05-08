# 08/05/2023
# Data Movement (-1.5,-1.5) to (1.5,1.5)
# Method : LQR
# Q = diag(75 75 35)
# R = diag(1 1 1)

data =[
    [-1.5000,-1.5000,0.0000],
    [-1.4888,-1.4876,-0.0142],
    [-1.4127,-1.4177,-0.0936],
    [-1.2658,-1.3108,-0.2353],
    [-1.1297,-1.2363,-0.3586],
    [-0.9891,-1.1685,-0.4844],
    [-0.8474,-1.0963,-0.6120],
    [-0.6985,-1.0154,-0.7391],
    [-0.5400,-0.9230,-0.8678],
    [-0.3708,-0.8190,-1.0034],
    [-0.1924,-0.7074,-1.1454],
    [-0.0318,-0.6048,-1.2729],
    [0.1130,-0.5078,-1.3881],
    [0.2462,-0.4112,-1.5005],
    [0.3721,-0.3151,-1.6120],
    [0.4950,-0.2187,-1.7248],
    [0.5969,-0.1351,-1.8217],
    [0.7396,-0.0140,-1.9579],
    [0.8628,0.0958,-2.0733],
    [0.9773,0.2041,-2.1803],
    [1.0738,0.3132,-2.2761],
    [1.1539,0.4212,-2.3623],
    [1.2203,0.5254,-2.4401],
    [1.2746,0.6234,-2.5100],
    [1.3191,0.7130,-2.5720],
    [1.3552,0.7948,-2.6285],
    [1.3845,0.8689,-2.6803],
    [1.4081,0.9355,-2.7274],
    [1.4270,0.9949,-2.7700],
    [1.4420,1.0477,-2.8082],
    [1.4537,1.0946,-2.8422],
    [1.4629,1.1364,-2.8724],
    [1.4701,1.1737,-2.8982],
    [1.4757,1.2071,-2.9206],
    [1.4801,1.2371,-2.9405],
    [1.4837,1.2639,-2.9578],
    [1.4867,1.2880,-2.9729],
    [1.4889,1.3098,-2.9870],
    [1.4908,1.3293,-2.9996],
    [1.4924,1.3469,-3.0110],
    [1.4937,1.3626,-3.0212],
    [1.4948,1.3768,-3.0303],
    [1.4957,1.3896,-3.0384],
    [1.4966,1.4011,-3.0456],
    [1.4973,1.4115,-3.0520],
    [1.4979,1.4208,-3.0578],
    [1.4985,1.4293,-3.0629],
    [1.4990,1.4368,-3.0675],
    [1.4995,1.4437,-3.0716],
    [1.4999,1.4498,-3.0752],
    [1.5003,1.4554,-3.0785],
    [1.5007,1.4604,-3.0813],
    [1.5010,1.4649,-3.0839],
    [1.5013,1.4690,-3.0862],
    [1.5016,1.4726,-3.0882],
    [1.5018,1.4759,-3.0900],
    [1.5021,1.4788,-3.0915],
    [1.5023,1.4814,-3.0929],
    [1.5024,1.4837,-3.0941],
    [1.5026,1.4858,-3.0952],
    [1.5028,1.4876,-3.0962],
    [1.5030,1.4892,-3.0971],
    [1.5031,1.4906,-3.0978],
    [1.5033,1.4918,-3.0985],
    [1.5035,1.4930,-3.0992],
    [1.5036,1.4940,-3.0998],
    [1.5037,1.4949,-3.1003],
    [1.5039,1.4956,-3.1007],
    [1.5040,1.4964,-3.1011],
    [1.5041,1.4970,-3.1014],
    [1.5042,1.4976,-3.1017],
    [1.5043,1.4981,-3.1019],
    [1.5043,1.4986,-3.1021],
    [1.5044,1.4991,-3.1023],
    [1.5045,1.4995,-3.1025],
    [1.5045,1.4998,-3.1026],
    [1.5046,1.5002,-3.1027],
    [1.5046,1.5005,-3.1027],
    [1.5047,1.5008,-3.1028],
    [1.5047,1.5011,-3.1029],
    [1.5048,1.5013,-3.1029],
    [1.5048,1.5015,-3.1030],
    [1.5048,1.5017,-3.1030],
    [1.5049,1.5019,-3.1030],
    [1.5049,1.5020,-3.1031],
    [1.5049,1.5022,-3.1031],
    [1.5050,1.5023,-3.1031],
    [1.5050,1.5024,-3.1031],
    [1.5050,1.5026,-3.1031]
]