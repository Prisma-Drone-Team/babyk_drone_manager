# Metriche di Odometria (OpenVINS)

**Impostazioni di tempo globali (sovrascritte se presenti in tempi.csv):** Start=inizio, Drop=N/D, End=fine

| experiment                    |   duration_s |   mean_3d_err_m |   rmse_3d_m | target_axes   |   target_rmse_overall_m |   target_rmse_before_m |   target_rmse_after_m |   improvement_% |
|:------------------------------|-------------:|----------------:|------------:|:--------------|------------------------:|-----------------------:|----------------------:|----------------:|
| baseline                      |        143.4 |          1.3504 |      1.675  | BL            |                  1.8757 |               nan      |              nan      |          nan    |
| baseline_condensatore_saltato |        110   |          1.2188 |      1.5589 | BL            |                  1.8027 |               nan      |              nan      |          nan    |
| baseline_nuova                |        139.2 |          1.4965 |      1.9733 | BL            |                  2.1288 |               nan      |              nan      |          nan    |
| cacabene                      |        141.1 |          1.3547 |      1.6739 | Z             |                  0.5657 |                 0.5903 |                0.4485 |           24.02 |
| cacabene2                     |        164.3 |          1.9424 |      2.3956 | Z             |                  0.5036 |                 0.5319 |                0.4347 |           18.26 |
| cacabene3                     |        143.4 |          1.0764 |      1.398  | Z             |                  0.1646 |                 0.1935 |                0.0344 |           82.24 |
| cacaprova1                    |        107.6 |          0.993  |      1.3231 | Z             |                  0.2759 |                 0.3139 |                0.2399 |           23.59 |
| cacaprova2                    |         99.9 |          0.789  |      1.0185 | Z             |                  0.4851 |                 0.4832 |                0.5253 |           -8.7  |
| completa_chiavica             |        221   |          1.2935 |      1.6831 | 3D            |                  1.3351 |                 1.1447 |                1.3983 |          -22.15 |
| completa_godo                 |        125.6 |          0.9134 |      1.1032 | 3D            |                  1.221  |                 1.2856 |                1.0871 |           15.44 |
| completa_non_pitta_buona      |        199.7 |          0.7937 |      0.9109 | 3D            |                  0.9278 |                 0.9002 |                0.9394 |           -4.36 |
| completa_non_pitta_meh        |        201.7 |          1.3904 |      1.7    | 3D            |                  1.5869 |                 1.4221 |                1.678  |          -18    |
| completa_ottima_da_tagliare   |        218.9 |          1.1528 |      1.2809 | 3D            |                  1.2949 |                 1.2287 |                1.3266 |           -7.97 |
| first                         |        133.9 |          1.2662 |      1.6275 | BL            |                  1.753  |               nan      |              nan      |          nan    |
| second                        |        147.8 |          1.586  |      2.0555 | BL            |                  2.2034 |               nan      |              nan      |          nan    |
| swipebene                     |        140.3 |          0.9206 |      1.1613 | XY            |                  1.0689 |                 1.2532 |                0.6706 |           46.49 |
