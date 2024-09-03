#In this file we define a sinngle ndt cell message
bool hasGaussian_    #Tell if the cell was only initialized and has no gaussian
float64 center_x       #x coordinate of center used for uninitilaised cells
float64 center_y       #y coordinate of center used for uninitilaised cells
float64 center_z       #z coordinate of center used for uninitilaised cells
float64 mean_x       #x coordinate of mean
float64 mean_y       #y coordinate of mean
float64 mean_z       #z coordinate of mean
float64[] cov_matrix #covariance matrix
float64 occupancy    #occupancy
float64 occupancy_raw    #occupancy
float64 red     #red color value
float64 green   #green color value
float64 blue    #blue color value
int64 label   #semantic class
int64 label_weight  #semantic label weight
int64 label_time    #semantic label timestamp
int64 N              #point count