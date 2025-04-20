import matplotlib.pyplot as plt
import matplotlib.patches as patches  # 👈 this is the missing piece
import re

# Paste your raw values here as a triple-quoted string
values = """
pos_x: 0.0000, pos_y: 0.0000, error: 0.0000, omega: 0.0000
pos_x: 0.0017, pos_y: -0.0014, error: 0.0000, omega: -0.0002
pos_x: 0.0017, pos_y: -0.0014, error: 0.0000, omega: -0.0002
pos_x: 0.0088, pos_y: -0.0068, error: -0.0001, omega: 0.0048
pos_x: 0.0088, pos_y: -0.0068, error: -0.0001, omega: 0.0048
pos_x: 0.0182, pos_y: -0.0139, error: 0.0000, omega: -0.0008
pos_x: 0.0182, pos_y: -0.0139, error: 0.0000, omega: -0.0008
pos_x: 0.0335, pos_y: -0.0256, error: 0.0008, omega: -0.0396
pos_x: 0.0335, pos_y: -0.0256, error: 0.0008, omega: -0.0396
pos_x: 0.0568, pos_y: -0.0415, error: 0.0019, omega: -0.0934
pos_x: 0.0568, pos_y: -0.0415, error: 0.0019, omega: -0.0934
pos_x: 0.0769, pos_y: -0.0539, error: 0.0028, omega: -0.1412
pos_x: 0.0769, pos_y: -0.0539, error: 0.0028, omega: -0.1412
pos_x: 0.0927, pos_y: -0.0635, error: 0.0043, omega: -0.2168
pos_x: 0.0927, pos_y: -0.0635, error: 0.0043, omega: -0.2168
pos_x: 0.1087, pos_y: -0.0728, error: 0.0061, omega: -0.3031
pos_x: 0.1087, pos_y: -0.0728, error: 0.0061, omega: -0.3031
pos_x: 0.1243, pos_y: -0.0814, error: 0.0079, omega: -0.3951
pos_x: 0.1243, pos_y: -0.0814, error: 0.0079, omega: -0.3951
pos_x: 0.1379, pos_y: -0.0889, error: 0.0100, omega: -0.5000
pos_x: 0.1379, pos_y: -0.0889, error: 0.0100, omega: -0.5000
pos_x: 0.1530, pos_y: -0.0967, error: 0.0124, omega: -0.6200
pos_x: 0.1530, pos_y: -0.0967, error: 0.0124, omega: -0.6200
pos_x: 0.1683, pos_y: -0.1041, error: 0.0149, omega: -0.7461
pos_x: 0.1683, pos_y: -0.1041, error: 0.0149, omega: -0.7461
pos_x: 0.1838, pos_y: -0.1112, error: 0.0176, omega: -0.8781
pos_x: 0.1838, pos_y: -0.1112, error: 0.0176, omega: -0.8781
pos_x: 0.1988, pos_y: -0.1176, error: 0.0202, omega: -1.0091
pos_x: 0.1988, pos_y: -0.1176, error: 0.0202, omega: -1.0091
pos_x: 0.2137, pos_y: -0.1240, error: 0.0233, omega: -1.1628
pos_x: 0.2137, pos_y: -0.1240, error: 0.0233, omega: -1.1628
pos_x: 0.2294, pos_y: -0.1306, error: 0.0269, omega: -1.3470
pos_x: 0.2294, pos_y: -0.1306, error: 0.0269, omega: -1.3470
pos_x: 0.2466, pos_y: -0.1375, error: 0.0311, omega: -1.5525
pos_x: 0.2466, pos_y: -0.1375, error: 0.0311, omega: -1.5525
pos_x: 0.2635, pos_y: -0.1428, error: 0.0343, omega: -1.7152
pos_x: 0.2635, pos_y: -0.1428, error: 0.0343, omega: -1.7152
pos_x: 0.2807, pos_y: -0.1473, error: 0.0373, omega: -1.8633
pos_x: 0.2807, pos_y: -0.1473, error: 0.0373, omega: -1.8633
pos_x: 0.2974, pos_y: -0.1508, error: 0.0398, omega: -1.9908
pos_x: 0.2974, pos_y: -0.1508, error: 0.0398, omega: -1.9908
pos_x: 0.3134, pos_y: -0.1537, error: 0.0423, omega: -2.1163
pos_x: 0.3134, pos_y: -0.1537, error: 0.0423, omega: -2.1163
pos_x: 0.3288, pos_y: -0.1557, error: 0.0444, omega: -2.2212
pos_x: 0.3288, pos_y: -0.1557, error: 0.0444, omega: -2.2212
pos_x: 0.3443, pos_y: -0.1566, error: 0.0459, omega: -2.2935
pos_x: 0.3443, pos_y: -0.1566, error: 0.0459, omega: -2.2935
pos_x: 0.3591, pos_y: -0.1571, error: 0.0473, omega: -2.3641
pos_x: 0.3591, pos_y: -0.1571, error: 0.0473, omega: -2.3641
pos_x: 0.3724, pos_y: -0.1569, error: 0.0483, omega: -2.4131
pos_x: 0.3724, pos_y: -0.1569, error: 0.0483, omega: -2.4131
pos_x: 0.3887, pos_y: -0.1558, error: 0.0491, omega: -2.4563
pos_x: 0.3887, pos_y: -0.1558, error: 0.0491, omega: -2.4563
pos_x: 0.4034, pos_y: -0.1541, error: 0.0496, omega: -2.4814
pos_x: 0.4034, pos_y: -0.1541, error: 0.0496, omega: -2.4814
pos_x: 0.4180, pos_y: -0.1517, error: 0.0498, omega: -2.4914
pos_x: 0.4180, pos_y: -0.1517, error: 0.0498, omega: -2.4914
pos_x: 0.4360, pos_y: -0.1478, error: 0.0498, omega: -2.4878
pos_x: 0.4360, pos_y: -0.1478, error: 0.0498, omega: -2.4878
pos_x: 0.4510, pos_y: -0.1434, error: 0.0491, omega: -2.4545
pos_x: 0.4510, pos_y: -0.1434, error: 0.0491, omega: -2.4545
pos_x: 0.4696, pos_y: -0.1363, error: 0.0475, omega: -2.3738
pos_x: 0.4696, pos_y: -0.1363, error: 0.0475, omega: -2.3738
pos_x: 0.4906, pos_y: -0.1290, error: 0.0469, omega: -2.3473
pos_x: 0.4906, pos_y: -0.1290, error: 0.0469, omega: -2.3473
pos_x: 0.5098, pos_y: -0.1212, error: 0.0463, omega: -2.3128
pos_x: 0.5098, pos_y: -0.1212, error: 0.0463, omega: -2.3128
pos_x: 0.5275, pos_y: -0.1119, error: 0.0445, omega: -2.2234
pos_x: 0.5275, pos_y: -0.1119, error: 0.0445, omega: -2.2234
pos_x: 0.5438, pos_y: -0.1017, error: 0.0421, omega: -2.1034
pos_x: 0.5438, pos_y: -0.1017, error: 0.0421, omega: -2.1034
pos_x: 0.5596, pos_y: -0.0907, error: 0.0394, omega: -1.9703
pos_x: 0.5596, pos_y: -0.0907, error: 0.0394, omega: -1.9703
pos_x: 0.5754, pos_y: -0.0785, error: 0.0364, omega: -1.8188
pos_x: 0.5754, pos_y: -0.0785, error: 0.0364, omega: -1.8188
pos_x: 0.5892, pos_y: -0.0661, error: 0.0329, omega: -1.6447
pos_x: 0.5892, pos_y: -0.0661, error: 0.0329, omega: -1.6447
pos_x: 0.6026, pos_y: -0.0534, error: 0.0295, omega: -1.4773
pos_x: 0.6026, pos_y: -0.0534, error: 0.0295, omega: -1.4773
pos_x: 0.6166, pos_y: -0.0381, error: 0.0250, omega: -1.2520
pos_x: 0.6166, pos_y: -0.0381, error: 0.0250, omega: -1.2520
pos_x: 0.6293, pos_y: -0.0227, error: 0.0204, omega: -1.0219
pos_x: 0.6293, pos_y: -0.0227, error: 0.0204, omega: -1.0219
pos_x: 0.6418, pos_y: -0.0052, error: 0.0147, omega: -0.7364
pos_x: 0.6418, pos_y: -0.0052, error: 0.0147, omega: -0.7364
pos_x: 0.6542, pos_y: 0.0132, error: 0.0091, omega: -0.4547
pos_x: 0.6542, pos_y: 0.0132, error: 0.0091, omega: -0.4547
pos_x: 0.6649, pos_y: 0.0309, error: 0.0036, omega: -0.1802
pos_x: 0.6649, pos_y: 0.0309, error: 0.0036, omega: -0.1802
pos_x: 0.6752, pos_y: 0.0480, error: -0.0010, omega: 0.0495
pos_x: 0.6752, pos_y: 0.0480, error: -0.0010, omega: 0.0495
pos_x: 0.6855, pos_y: 0.0660, error: -0.0055, omega: 0.2745
pos_x: 0.6855, pos_y: 0.0660, error: -0.0055, omega: 0.2745
pos_x: 0.6961, pos_y: 0.0847, error: -0.0093, omega: 0.4649
pos_x: 0.6961, pos_y: 0.0847, error: -0.0093, omega: 0.4649
pos_x: 0.7080, pos_y: 0.1043, error: -0.0118, omega: 0.5891
pos_x: 0.7080, pos_y: 0.1043, error: -0.0118, omega: 0.5891
pos_x: 0.7195, pos_y: 0.1224, error: -0.0126, omega: 0.6290
pos_x: 0.7195, pos_y: 0.1224, error: -0.0126, omega: 0.6290
pos_x: 0.7314, pos_y: 0.1420, error: -0.0129, omega: 0.6472
pos_x: 0.7314, pos_y: 0.1420, error: -0.0129, omega: 0.6472
pos_x: 0.7429, pos_y: 0.1601, error: -0.0117, omega: 0.5870
pos_x: 0.7429, pos_y: 0.1601, error: -0.0117, omega: 0.5870
pos_x: 0.7559, pos_y: 0.1817, error: -0.0097, omega: 0.4873
pos_x: 0.7559, pos_y: 0.1817, error: -0.0097, omega: 0.4873
pos_x: 0.7670, pos_y: 0.2000, error: -0.0070, omega: 0.3513
pos_x: 0.7670, pos_y: 0.2000, error: -0.0070, omega: 0.3513
pos_x: 0.7774, pos_y: 0.2162, error: -0.0033, omega: 0.1673
pos_x: 0.7774, pos_y: 0.2162, error: -0.0033, omega: 0.1673
pos_x: 0.7889, pos_y: 0.2325, error: 0.0021, omega: -0.1072
pos_x: 0.7889, pos_y: 0.2325, error: 0.0021, omega: -0.1072
pos_x: 0.8018, pos_y: 0.2497, error: 0.0093, omega: -0.4630
pos_x: 0.8018, pos_y: 0.2497, error: 0.0093, omega: -0.4630
pos_x: 0.8141, pos_y: 0.2654, error: 0.0169, omega: -0.8460
pos_x: 0.8141, pos_y: 0.2654, error: 0.0169, omega: -0.8460
pos_x: 0.8273, pos_y: 0.2814, error: 0.0258, omega: -1.2908
pos_x: 0.8273, pos_y: 0.2814, error: 0.0258, omega: -1.2908
pos_x: 0.8393, pos_y: 0.2974, error: 0.0340, omega: -1.6999
pos_x: 0.8393, pos_y: 0.2974, error: 0.0340, omega: -1.6999
pos_x: 0.8521, pos_y: 0.3155, error: 0.0431, omega: -2.1562
pos_x: 0.8521, pos_y: 0.3155, error: 0.0431, omega: -2.1562
pos_x: 0.8637, pos_y: 0.3336, error: 0.0516, omega: -2.5776
pos_x: 0.8637, pos_y: 0.3336, error: 0.0516, omega: -2.5776
pos_x: 0.8742, pos_y: 0.3531, error: 0.0594, omega: -2.9718
pos_x: 0.8742, pos_y: 0.3531, error: 0.0594, omega: -2.9718
pos_x: 0.8831, pos_y: 0.3718, error: 0.0664, omega: -3.3220
pos_x: 0.8831, pos_y: 0.3718, error: 0.0664, omega: -3.3220
pos_x: 0.8911, pos_y: 0.3910, error: 0.0731, omega: -3.6540
pos_x: 0.8911, pos_y: 0.3910, error: 0.0731, omega: -3.6540
pos_x: 0.8969, pos_y: 0.4101, error: 0.0783, omega: -3.9174
pos_x: 0.8969, pos_y: 0.4101, error: 0.0783, omega: -3.9174
pos_x: 0.9017, pos_y: 0.4310, error: 0.0832, omega: -4.1624
pos_x: 0.9017, pos_y: 0.4310, error: 0.0832, omega: -4.1624
pos_x: 0.9030, pos_y: 0.4443, error: 0.0852, omega: -4.2618
pos_x: 0.9030, pos_y: 0.4443, error: 0.0852, omega: -4.2618
pos_x: 0.8993, pos_y: 0.4563, error: 0.0832, omega: -4.1600
pos_x: 0.8993, pos_y: 0.4563, error: 0.0832, omega: -4.1600
pos_x: 0.8935, pos_y: 0.4608, error: 0.0787, omega: -3.9367
pos_x: 0.8935, pos_y: 0.4608, error: 0.0787, omega: -3.9367
pos_x: 0.8855, pos_y: 0.4626, error: 0.0722, omega: -3.6084
pos_x: 0.8855, pos_y: 0.4626, error: 0.0722, omega: -3.6084
pos_x: 0.8663, pos_y: 0.4624, error: 0.0554, omega: -2.7713
pos_x: 0.8663, pos_y: 0.4624, error: 0.0554, omega: -2.7713
pos_x: 0.8457, pos_y: 0.4602, error: 0.0366, omega: -1.8287
pos_x: 0.8457, pos_y: 0.4602, error: 0.0366, omega: -1.8287
pos_x: 0.8247, pos_y: 0.4559, error: 0.0163, omega: -0.8148
pos_x: 0.8247, pos_y: 0.4559, error: 0.0163, omega: -0.8148
pos_x: 0.8040, pos_y: 0.4500, error: -0.0047, omega: 0.2330
pos_x: 0.8040, pos_y: 0.4500, error: -0.0047, omega: 0.2330
pos_x: 0.7814, pos_y: 0.4429, error: -0.0287, omega: 1.4352
pos_x: 0.7814, pos_y: 0.4429, error: -0.0287, omega: 1.4352
pos_x: 0.7610, pos_y: 0.4365, error: -0.0515, omega: 2.5735
pos_x: 0.7610, pos_y: 0.4365, error: -0.0515, omega: 2.5735
pos_x: 0.7418, pos_y: 0.4310, error: -0.0737, omega: 3.6830
pos_x: 0.7418, pos_y: 0.4310, error: -0.0737, omega: 3.6830
pos_x: 0.7231, pos_y: 0.4261, error: -0.0960, omega: 4.7999
pos_x: 0.7231, pos_y: 0.4261, error: -0.0960, omega: 4.7999
pos_x: 0.6930, pos_y: 0.4229, error: -0.1337, omega: 6.6845
pos_x: 0.6930, pos_y: 0.4229, error: -0.1337, omega: 6.6845
pos_x: 0.6643, pos_y: 0.4263, error: -0.1712, omega: 8.5588
pos_x: 0.6643, pos_y: 0.4263, error: -0.1712, omega: 8.5588
pos_x: 0.6490, pos_y: 0.4353, error: -0.1909, omega: 9.5432
pos_x: 0.6490, pos_y: 0.4353, error: -0.1909, omega: 9.5432
pos_x: 0.6449, pos_y: 0.4518, error: -0.1930, omega: 9.6504
pos_x: 0.6449, pos_y: 0.4518, error: -0.1930, omega: 9.6504
pos_x: 0.6500, pos_y: 0.4649, error: -0.1823, omega: 9.1150
pos_x: 0.6500, pos_y: 0.4649, error: -0.1823, omega: 9.1150
pos_x: 0.6588, pos_y: 0.4729, error: -0.1680, omega: 8.4000
pos_x: 0.6588, pos_y: 0.4729, error: -0.1680, omega: 8.4000
pos_x: 0.6693, pos_y: 0.4765, error: -0.1531, omega: 7.6564
pos_x: 0.6693, pos_y: 0.4765, error: -0.1531, omega: 7.6564
pos_x: 0.6803, pos_y: 0.4762, error: -0.1391, omega: 6.9529
pos_x: 0.6803, pos_y: 0.4762, error: -0.1391, omega: 6.9529
pos_x: 0.6878, pos_y: 0.4730, error: -0.1306, omega: 6.5281
pos_x: 0.6878, pos_y: 0.4730, error: -0.1306, omega: 6.5281
pos_x: 0.6930, pos_y: 0.4688, error: -0.1252, omega: 6.2594
pos_x: 0.6930, pos_y: 0.4688, error: -0.1252, omega: 6.2594
pos_x: 0.6967, pos_y: 0.4642, error: -0.1218, omega: 6.0875
pos_x: 0.6967, pos_y: 0.4642, error: -0.1218, omega: 6.0875
pos_x: 0.6991, pos_y: 0.4596, error: -0.1199, omega: 5.9933
pos_x: 0.6991, pos_y: 0.4596, error: -0.1199, omega: 5.9933
pos_x: 0.7002, pos_y: 0.4560, error: -0.1193, omega: 5.9627
pos_x: 0.7002, pos_y: 0.4560, error: -0.1193, omega: 5.9627
pos_x: 0.7009, pos_y: 0.4524, error: -0.1192, omega: 5.9585
pos_x: 0.7009, pos_y: 0.4524, error: -0.1192, omega: 5.9585
pos_x: 0.7012, pos_y: 0.4472, error: -0.1198, omega: 5.9889
pos_x: 0.7012, pos_y: 0.4472, error: -0.1198, omega: 5.9889
pos_x: 0.7006, pos_y: 0.4421, error: -0.1214, omega: 6.0705
pos_x: 0.7006, pos_y: 0.4421, error: -0.1214, omega: 6.0705
pos_x: 0.6993, pos_y: 0.4378, error: -0.1237, omega: 6.1832
pos_x: 0.6993, pos_y: 0.4378, error: -0.1237, omega: 6.1832
pos_x: 0.6975, pos_y: 0.4338, error: -0.1266, omega: 6.3307
pos_x: 0.6975, pos_y: 0.4338, error: -0.1266, omega: 6.3307
pos_x: 0.6946, pos_y: 0.4295, error: -0.1308, omega: 6.5409
pos_x: 0.6946, pos_y: 0.4295, error: -0.1308, omega: 6.5409
pos_x: 0.6895, pos_y: 0.4241, error: -0.1380, omega: 6.8991
pos_x: 0.6895, pos_y: 0.4241, error: -0.1380, omega: 6.8991
pos_x: 0.6814, pos_y: 0.4189, error: -0.1491, omega: 7.4556
pos_x: 0.6814, pos_y: 0.4189, error: -0.1491, omega: 7.4556
pos_x: 0.6706, pos_y: 0.4162, error: -0.1637, omega: 8.1868
pos_x: 0.6706, pos_y: 0.4162, error: -0.1637, omega: 8.1868
pos_x: 0.6574, pos_y: 0.4179, error: -0.1817, omega: 9.0847
pos_x: 0.6574, pos_y: 0.4179, error: -0.1817, omega: 9.0847
pos_x: 0.6468, pos_y: 0.4246, error: -0.1958, omega: 9.7884
pos_x: 0.6468, pos_y: 0.4246, error: -0.1958, omega: 9.7884
pos_x: 0.6395, pos_y: 0.4357, error: -0.2044, omega: 10.2185
pos_x: 0.6395, pos_y: 0.4357, error: -0.2044, omega: 10.2185
pos_x: 0.6373, pos_y: 0.4511, error: -0.2039, omega: 10.1955
pos_x: 0.6373, pos_y: 0.4511, error: -0.2039, omega: 10.1955
pos_x: 0.6430, pos_y: 0.4656, error: -0.1917, omega: 9.5833
pos_x: 0.6430, pos_y: 0.4656, error: -0.1917, omega: 9.5833
pos_x: 0.6560, pos_y: 0.4740, error: -0.1713, omega: 8.5629
pos_x: 0.6560, pos_y: 0.4740, error: -0.1713, omega: 8.5629
pos_x: 0.6708, pos_y: 0.4749, error: -0.1516, omega: 7.5802
pos_x: 0.6708, pos_y: 0.4749, error: -0.1516, omega: 7.5802
pos_x: 0.6816, pos_y: 0.4699, error: -0.1393, omega: 6.9648
pos_x: 0.6816, pos_y: 0.4699, error: -0.1393, omega: 6.9648
pos_x: 0.6881, pos_y: 0.4628, error: -0.1329, omega: 6.6454
pos_x: 0.6881, pos_y: 0.4628, error: -0.1329, omega: 6.6454
pos_x: 0.6917, pos_y: 0.4547, error: -0.1302, omega: 6.5093
pos_x: 0.6917, pos_y: 0.4547, error: -0.1302, omega: 6.5093
pos_x: 0.6925, pos_y: 0.4481, error: -0.1305, omega: 6.5239
pos_x: 0.6925, pos_y: 0.4481, error: -0.1305, omega: 6.5239
pos_x: 0.6920, pos_y: 0.4429, error: -0.1321, omega: 6.6031
pos_x: 0.6920, pos_y: 0.4429, error: -0.1321, omega: 6.6031
pos_x: 0.6910, pos_y: 0.4394, error: -0.1339, omega: 6.6962
pos_x: 0.6910, pos_y: 0.4394, error: -0.1339, omega: 6.6962
pos_x: 0.6893, pos_y: 0.4353, error: -0.1368, omega: 6.8375
pos_x: 0.6893, pos_y: 0.4353, error: -0.1368, omega: 6.8375
pos_x: 0.6870, pos_y: 0.4315, error: -0.1403, omega: 7.0128

"""  # replace with your full data

x, y = 0.31, 0.39
radius = 0.5

'''
import math
radius = 0.5
theta = 0
x = radius * math.cos(theta + math.pi / 2)
y = radius * math.sin(theta + math.pi / 2)
print(math.hypot(0 - x, 0 - y))
'''

# Extract pos_x and pos_y values using regex
x_values = [float(match) for match in re.findall(r"pos_x:\s*(-?\d+\.\d+)", values)]
y_values = [float(match) for match in re.findall(r"pos_y:\s*(-?\d+\.\d+)", values)]

# Plotting
fig, ax = plt.subplots(figsize=(6, 6))
ax.plot(x_values, y_values, marker='o', linestyle='-', color='blue', label='Path')
ax.scatter(x, y, color='red', s=100, label='Target Point')

# Add a circle around the point
circle = patches.Circle((x, y), radius, fill=False, edgecolor='red', linestyle='--', linewidth=2, label='Radius')
ax.add_patch(circle)

# Final plot setup
ax.set_title('p=-50,r=0.5, new error, added default rot vel\nbest yet, but still ends up too far and circles')
ax.set_xlabel('pos_x')
ax.set_ylabel('pos_y')
ax.grid(True)
ax.axis('equal')
ax.legend()
plt.tight_layout()
plt.show()