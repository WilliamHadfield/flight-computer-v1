# understand allan #


import json # read/write json data parses json files into python lists/dicts or converts them back to JSON #
import re # regular expressions the text-based search engine used to find characters/text strings which are then used in someway #
import sys # talk to the running interpreter eg the shell command and then you can give it specfic overiding commands like to stop the programe as a simple one or put the outputs of the file somewhere or create a new file etc #
import allantools # compute allan variance/deviation for characterizing the sensor noise #
import matplotlib.pyplot as plt # useful for plotting or making graphs and charts #
import numpy as np # numerical arrays and math operation on data , an equivalent rust naglebra #

LOG_PATH = "data/sensor-log.txt" # where our data is coming from 
OUT_DIR = "output" # the directory where our output is written to or stored as outputs so that we can retrieve/extract/go through it 

WARMUP_TRIM_S = 0.0 # number of seconds to disregard at the startup of sensors as intially they can be unpredictiablly noisy and skew results ADJUST THIS VALUE 

TAU_MAX_DIV = 10.0 # essentially the cap on the maximum bucket pieces that you can split the total dataset up into. so 10.0 represent 1/10 or 10 buckets of data of tau seconds (which values to try) or 
# the max range of what buckets you can do #

NS_MIN = 10 # this is fundamentally the same sort of check as TAU_MAX but slightly different its for quality assurance its so that your allan deviation has a minimum confidence level that the error only 0.3 or some N value 
# above 10 clusers

REL_ERR_MAX = 0.30 # this is the definiable and hardcoded max error acceptable see its the finale "check" in that if there noise added, this could be the last distingiushing check that hardcaps the error at 30 percent and no more 

SLOPE_TOL_WHITE = 0.12 # tolerance for the white noise part of the allan deviance curve, its set to 0.12 +- outside of the expect -0.5 eg outside this range the algorithm wont accept the data as legimate.

SLOPE_TOL_RRW = 0.15 # tolerance for Random walk part of the allan deviance graph/curve, set to 0.15 eg meaning it will fall +- 0.15 outside the expected value.

SLOPE_TOL_FLOOR = 0.12 # same tolerance as white noise because theoritcally this is the point where white noise and rrw has been seperated and now afterwards 
# can just observe RRW effects which should be about 0, so its the lowest the noise can theoritcally go so therefore it needs the 0.12 to be consitent with the white noise assumption

MIN_REGION_POINTS = 4.0 # N consectuive tau values inside a region (white noise stretch of the curve for example) that stays within the tolerance level (0.12 for white noise)

PTS_PER_DECADE = 20 # number of samples needed for each logarithm tau band, or for ever factor of 10 seconds eg 0.1 -> 1  20 values, 1 -> 10, 20 values, 10 -> 100 20 values etc.

RESID_WARN = 0.05 # essentially measures the total distribution of the plotted points and if they any plot is 0.5 +- outside the tolerance than it will send a warning out, essentially desgined to spot outliers
# eg measuring how much the scatter is outside the ideal or average in this case 5 percent and if above this threshold warn

JITTER_WARN = 0.01 # excat sampling rates is never perfect (N hz) so Jitter_warning is just the percentage of flucation that your sampling rates have which should for right now be less than 1 percent which is sound
# as sampling rates are critically relevant to an accurate allan deviation.

GAP_FACTOR = 5.0 # the dt gap between warning and not something of note, eg how many dts have to elapse before non sampling is a problem or if a sample arrives at 14.8hz but then suddenly becomes 5x slower for one sample to arrive
# or time elapsed is N times longer than the usual than gap_factor will flag it.

T_MIN_RRW_S = 1800.0 # minimum time for random walk to be observable.

BI_SCALE = 0.664 # the number that is constant attained by pushing the flicker spectrum through the allan deviation operator.

ANSI = re.compile(r"\x1b\[[0-9;]*[a-zA-Z]") # a kinda macro to avoid recompiling this code at each use, essentially its just destrips terminal code formatting metacode from every line to get it ready for parsing.

def parse_log(path): # the normal function definition for a python function, this function is called parse_log and it takes a parameter path
    imu = [] # a list is a generic growable vector essentially the wildcard of vectors both growable/typless and holds nothing currently
    with open(path, "r") as f: # open the file at path in read mode or read only permissions like a libarary card you can only read and take the books not edit them.
        for line in f: # just iterates over each line in the file
            line = ANSI.sub("", line).strip() # uses the metacode destripping macro that removes the regux colour metacode associated with each data line from the terminal, then it strips the whitespace from each line
            # binding it back to line afterwards with the polished result.
            if "INFO" not in line or "ImuData" not in line: # essentially searches for INFO or IMUDATA throughout the dataset. you need both just incase ImuData overflows to the next line.
                continue # skips over lines not containg INFO or IMuData because there irrelvant to calculating allan devaition
            ts = float(line.split()[0]) # essentially this line of code just splits the timestamp off the start of each IMuData line cause of the indexing at 0 and binds it to ts we 
            # timestamp is useful for a few reasons ensures we have hz rate, can check for gaps or dropped samples and stabillity. also to double check averaging time because allans variance is heavily based on tau
            vals = [
                 float(re.search(rf"{k}:\s*([-\d.]+)", line).group(1)) # regex s used to jump to the next key phrase eg gyro_x and then it recordes the number listed after the gyro_x (the data value) and then parses it to vals.
                for k in ("gyro_x", "gyro_y", "gyro_z",   # cycling through each of the different values we need to measure.
                          "accel_x", "accel_y", "accel_z")
            ]
            imu.append([ts] + vals) # simply recombines the timestamp with the sliced data values.
    if not imu:
        sys.exit(f"no ImuData lines found in {path}") # not condition eg if the value line isnt imu this prevents the array from returning a 0 sized array which could cause errors if propogated forward instead of handled.
    return np.array(imu) # returns the array with the data needed

def check_sampling(ts, label="IMU"): # a function defintion that takes in a ts variable and a static string slice like type of "IMU"
    warnings = [] # this defines a list (rusts vector but is typless and growable and currently holds nothing)
    order = np.argsort(ts, kind = 'stable') # essentially is a numpy workaround for sorting essentially rather than sort you calculate the order of items if sorted asecedingly then return the indexs of them so eg 10,50,30,70,40 would return 0 2 4 1 3 eg the ascending order of indexs
    # this also allows us to keep values of the same magnitude in the same equivalent order rather than changing there order which matters which is why we have kind = 'stable'
    if not np.array_equal(order, np.arrang(len(ts))): # checks the chronolgical consitency of the timestamps to make sure there in order- NOTE this however wont check if two timestamps happen at the same time. so be wary of that
        warnings.append("timestamps not monotonic; data was re-sorted")
    ts = ts[order] # arranges the timestamp chronolgically but if a value is equal than its order is kept constient with what it is before so it rearranges the list to every value is equal or above the previous one
    dup = np.sum(np.diff(ts) == 0) # counts the adjacents timestamps and puts the total count into a container called dup
    if dup:
        warnings.append(f"{dup} duplicate timestamps") # if any duplicate timestamps warn the system or throw a warning
    dt = np.diff(ts) # gives you the full list of indvidual timestep gaps
    dt = dt[dt > 0] # so boolean masking of dt > 0 if dt isnt equal to 0 return false and dont get stored if above 0 get stored so this line just compiles a lsit of all the dt steps that are above 0
    med = np.median(dt) # finds the median timestep gap more accurate for noisy environment where you could get a 0.1 noise drop
    jitter = np.std(dt) / np.mean(dt) # absoulte spread of dt gaps / average gap of timestep which gives the relative unceranity or timing variation eg the random uncertanity of a value say its expected to be 10ms each timestep
    # but in practice it flucates between 9-11 this is what is calculated here.
    gaps = np.sum(dt > GAP_FACTOR * med) # so if a dt is bigger than 5 dts of the median sample store it then we warn later.
    if jitter > JITTER_WARN:
        warnings.append(f"sampling jitter {1000 * jitter:.1f}% of mean dt") # warns if jitter exceeds the jitter_warn threshold just some error handling
    if gaps:
        warnings.append(f"{gaps} gaps > {GAP_FACTOR:.0f} x median dt") # warns if gaps > 0
    
    fs = 1.0 / med # NOTE this is a option med or mean, i decied to go with med because its more accurate if there timing dropouts which in real hardware there likely is so choosing the med value under those cirmcustances is correct.
    T = ts[-1] - ts[0] # total elapsed time of the recording or of the data package
    print(f"[{label}] fs = {fs:.2f} Hz, T = {T:.0f} s, "
           f"median dt = {med*1e3:.2f} ms, jitter = {100*jitter:.2f}%") # diagnoistic info about certain charactersitics of the log like jitter rate
    
    if T < T_MIN_RRW_S:
         warnings.append(f"WARNING: record is {T:.0f} RRW is unlikely unoberservable") # checks to see if RRW is observable eg you have enough samples for it to be oberservable
    
    for w in warnings:
        print(f" WARNINGS: {w}") # prints out the warnings above if they happen


    return order,fs,T # returns the values.

def adev_masked(x,fs,T):
    tau_lo = 2.0 / fs # shortest time that you can form one tau out of it or one bucket of time.
    tau_hi = T/ TAU_MAX_DIV # maximum time that you can form one tau or one bucket of time.
    n_dec = np.log10(tau_hi / tau_lo) # measures decades or log10 increments between the lowest and highest tau
    taus_req = np.logspace(np.log10(tau_lo), np.log10(tau_hi)) # essentially this creates the bounds which your tau values sit, lowest at tau_lo and highest at tau_hi and then you will have 10x spacing between each based on pts per decade, the ration being 10 x (decades / (number of points per tau - 1))
    max(int(n_dec * PTS_PER_DECADE), 8) # essentially sets the hard minimum of tau points at 8, if n_dec * pts_per_decade is smaller than 8 then use 8 instead.
    taus, adevs, errs, ns = allantools.oadev( # this line actually runs the allan devation and it namely computes the overlapping allan devaition eg reuses data across overlapping windows rather than chopping into disjointed chunks.
        x, rate=fs, data_type="freq", taus=taus_req)
    # x -> your data, rate=fs -> sample rate, so allantools knows the real time spacing
    # data_type = "freq" tells x its frequency data (rate measurment - gyro output in degrees), taus = taus_req is the tau log grid we created just reiterating which time to evalulate
    ok = (ns >= NS_MIN) & (errs / adevs < REL_ERR_MAX) & (taus <= tau_hi) # quality mask ensures values are sane and accurate.
    return taus, adevs, errs, ns, ok # returns the 5 values to be used later
# taus,adves the curve itself you plot these and fit slopes to them
# errs -> error bars, for plotting or weighting fits
# ns -> sample counts, nice to have not entirely neccsary
# ok -> qualtiy mask it fundamentally contains a list of true/false flags that line up with the dataset and assess the reliabilit and accuracy for each tau.


def local_scope(taus, adevs): # function definition that accepts a taus and adevs variable
    s = np.gradient(np.log10(adevs), np.log10(taus)) # computes the gradient of the line at each point this essentially is to track which region we are in eg (-0.5 -> 0 white noise, 0 -> 0.5 the floor and 0.5 -> ... is Random walk)
    if len(s) >= 3: # check to see if the slope array has atleast 3 elements
        s = np.convolve(s, np.ones(3) / 3.0, mode = "same") # averages slope gradient over 3 tau points
    return s # returns this average

def contiguous_runs(mask): # function definition that accepts a mask variable the core point of this function is to find the continous tau points at each critical slope point -0.5 for white noise 0 for the floor and 0.5 for random walk and track how many consitent points in a row there are and return them
    runs, start = [], None # defining a list called runs and a none variable called start, dict = {} (hashmap) list = [] (super vector)
    for i, v in enumerate(mask): # tuple iteration from mask, eg i,v tuple is used at each iteration of mask, enumerate provides both the index and value at each iteration
        # this results in i and v binding to i = index and v = value.
        if v and start is None: # so this just means v is truthy and start is None, truthy meaning a (non false,None or zero value eg value >0, True, or like Some)
            start = i # sets start to the current index marking where the run begins
        
        elif not v and start is not None: # else if = elif, v is falsy and start isnt none or eg the other condition
            runs.append((start,i)) # gives the running total of true tau points in a row,
            start = None # resets the run
    
    if start is not None: # basically is the last check, just make sure if a run is still open, close and append it.
        runs.append((start, len(mask))) # last check just appends the final run span, boundary pair for the one run that happened to not be closed properly.
    
    return runs # returns all appended runs data to be used later

def pick_region(slopes, ok, target, tol, prefer): # function defintion that takes slopes ok target tol and prefer variable.
    mask = ok & (np.abs(slopes - target) < tol) # just checks wether its a valid value and also that the difference between the slope value and the target slope value doesnt exceed the threshold.
    runs = [r for r in contiguous_runs(mask) if r[1] - r[0] >= MIN_REGION_POINTS ] # essentially filters run downs to long enough points that exceed the minimum regional points or have a certain number of tau points for it to be valid and accurate.
    if not runs: # checks wether runs is empty
        return None # exits the function
    return runs[0] if prefer == "first" else runs[-1] # picks one run to return, the first if pefer first otherwise the last, runs[0] to runs[-1] or the last run all qualify
# returns the first avaiable passed run if prefer variable = "first"


def composite_fit(taus,adevs, errs, ok): # function definition taking a taus adevs err and ok parameter.
    t = taus[ok] # boolean mask indexing returns the tau value when and if ok is equal to true.
    y = adevs[ok] ** 2 # boolean mask indexing again (returns the tau value when ok is true) but then squares it to convert it to variance.
    sig = 2.0 * adevs[ok] * errs[ok] # propogating uncertanity from allan deviation to variance.
    w = 1.0 / np.maximum(sig, 1e-300) # this builds the weights as the inverse of uncertanity, it also has a built in zero defence mechanism or near zero where past a certain value it will default and assume a fixed 1e-300 to prevent inf or NaN values.
    A = np.column_stack([1.0 / t, np.ones_like(t), t / 3.0]) * w[:, None] # so three models interwieving white noise curve, floor curve and RRW walk curve and then you times by the weight calculated by inverting the unceranity or using a fixed value if too small giving you the weighted design matrix eg same three curve shapes just each tau point scaled by weight.
    b = y * w # calculating the weighted measured allan variance or b
    try: # wrapping the NNLS solver to catch unexpected runtime matrix errors
        from scipy.optimize import nnls # importing the NNLS solver.
        coef, _ = nnls(A, b) # runs the NNLS solve, and unpacks the result, it returns the coefficent vector and the residual norm, disregarding the residual norm in the process. (the coefficent vector being each coefficent for each value of c for each model or part of the curve (white noise, floor, RRW))
    except ImportError: # if the import failed for unresolved import issues or isnt avaible this is the backup.
        coef = np.maximum(np.linalg.lstsq(A, b, rcond=None)[0], 0.0) # fallback NLSS solver, does ordinary least squares solves A * c = b, it solves unconstrained and also floor negatives at zero.
        for _ in range(200): # for loop that runs 200 times/
            for k in range(3): # for loop tat run 3 times.
                r = b - A @ coef + A[:, k] * coef[k] # computes a partial resdiue which r is the slice of measured dat that portion is responsible for fitting the column k or the template for one portion of the total model or RRW,white noise or floor.
                coef[k] = max(np.dot(A[:, k], r) / np.dot(A[:, k], A[:, k]), 0.0) # least square solution for one coefficent or k column.

    n2, _, k2 = coef # collects both the white noise coefficent and the random walk coefficent throwing away the bias instabillity or floor coefficent.
    N = np.sqrt(n2) if n2 > 0 else np.nan # converts it back to standard deviation or if n2 is smaller than or equal to 0 eg negative it returns NaN
    K = np.sqrt(k2) if k2 > 0 else np.nan # converts it back to standard deviation or if k2 is smaller than or equal to 0 eg negative it returns NaN
    return K,N # returns the two standard deviation values

def fit_fixed_slope(taus, adevs, errs, region, m):
    i,j = region # unpackaging a tuple.
    x = np.log10(taus[i:j]) # slices a sub-range or just the region between i -> j of tau points. or averaging time points.
    y = np.log10(adevs[i:j]) # slices a sub-range or region between i -> j of tau points.
    sig = errs[i:j] /  (adevs[i:j] * np.log(10.0)) # converting per tau point uncertanity into log space for the region of i -> j
    w = 1.0 / np.maximum(sig, 1e-12) ** 2 # essentially the weighted points slightly alter the line of best fit, and adjust the line of best fit to the existing points, the lower the uncertanity the more pull or weight the point has over the line.
    c = np.sum(w * (y- m * x)) / np.sum(x) # finds the y intercept of the new line of best fit readjusted from the weighted points.
    resid = np.sqrt(np.mean((y - (m * x + c)) ** 2)) # this compares the new weighted readjusted line of best fit to the unweighted points in each region and calculates the difference.
    return 10 ** c, resid # returns the delogged allan deviation coefficent (eg only describes white noise, bias instabillity or random walk). and resid is the diagnoistic of how well the line fits the data. eg how trustworthy this regions coefficent is.

# axis extraction

def extract_axis(x, fs, T, label, unit):
    taus, adevs, errs, ns, ok = adev_masked(x, fs, T) # unpacks 5 values from adev_masked
    slopes = local_scope(taus, adevs) # gives you an array of gradients at each tau point on the curve.
    out = { "label" : label, "fs_hz" : fs, "unit" : unit, 
           "N" : np.nan, "K" : np.nan, "B" : np.nan, 
            "sigma_white_per_sample" : np.nan, "sigma_bias_step" : np.nan} # its essentially an equivalent of hashmap<string, f64> from rust. it outputs all the values at nan intially as a fail safe
    fits = {} # empty dictionary or a hashmap equivalent from rust.
    K_comp, N_comp = composite_fit(taus, adevs, errs, ok) # unweighrted curve goes in and weighted curve comes out, then calculated into two allan deviation coefficent representing white noise and random walk.
    reg = pick_region(slopes, ok, -0.5, SLOPE_TOL_WHITE, prefer = "first") # its finds the white noise start end tuple pair and binds it to reg or None if there nothing
    if reg is not None and np.isfinite(N_comp): # just a check to make sure there are reg values and they arent inf or NaN hence the np.isfinite
        N_slope, resid = fit_fixed_slope(taus, adevs, errs, regs, -0.5) # obtains the delogged allan deviation coefficent for the white noise section of the curve and the residue eg the diagionstic of how well the line confomred to the tau data points. resid is RMS
        out["N"] = N_comp # rewrites the white noise coefficent as the actual white noise allan deviation coefficent rewriting over the NaN temporary placeholder value
        out["Sigma_white_per_sample"] = N_comp * np.sqrt(fs) # N_comp is the white noise coefficent multiplying by the sqrt freqency converts it to per sample standard deviation of white noise, hence sigma white per sample
        fits["White"] = reg, N_comp # records the white region and composite NNLS into the fits hashmap.
        if abs(N_slope / N_comp - 1.0) > 0.20: # compares the white noise aspect of NNLS and N_slope and checking if there 20 percent different
            print("white-noise estimators disagree")
        
        if resid > RESID_WARN: # checks residue isnt above the bounds.
            print("white-noise region residual")
        
    else:
         print(" WARNING: no -1/2 slope region found")


    # NOTE why to get the true unlogged allan deviation coefficent do i need to x the rrw section by square root 3????

    reg = pick_region(slopes, ok, +0.5,  SLOPE_TOL_RRW, "last" ) # finds the random walk start and end tuple pair and bind it to reg or None if there nothing.
    if reg is not None and np.isfinite(K_comp): # just a check to make sure the reg values are real and also they arent inf or NaN
        v1, resid = fit_fixed_slope(taus, adevs, errs, reg, +0.5) # obtains the delogged allan deviation coefficent for the random walk section of the curve and also the residue eg the diagnoistic information.
        K_slope = v1 * np.sqrt(3.0) # fixed scalar applied to random walk coefficent to ensure correctness
        out["K"] = K_comp # rewrites the random walk noise coefficent as the actual random walk allan deviation coefficent, rewriting over the NaN value.
        out["sigma_bias_step"] = K_comp / np.sqrt(fs) # per sample standard deviation of the bias random walk computed from the composite RRW coefficent NOTE check the validatity of this section.
        fits["rrw"] = (reg, K_comp / np.sqrt(3.0)) # records the RRW fit into the fits hashmap, which squared is also coidentently the per step bias state process variance in Q or process noise. because we use variances in matrix's.
        if abs(K_slope / K_comp - 1.0) > 0.25: # same as above compares the RRW NNLS with the K_slope to check if there inside the 25 percent bounds.
            print(f"[{label}] WARNING: RRW estimators disagree ")
        
        if resid > RESID_WARN: # checks residue.
            print(f"  [{label}] WARNING: RRW region residual bad")

    else:
        print(f"  [{label}] WARNING: no +1/2 slope region found; K ")
        
    reg = pick_region(slopes,ok,0.0,SLOPE_TOL_FLOOR, prefer = "first") # finds the bias instabillity or the floor start and end tuple pair and bind it to reg or None if there nothing.
    if reg is not None: # just a check to make sure there are reg values and they arent inf or NaN hence the np.isfinite
        i, j = reg # unpacks the bias instabillity/floor index pair into i,j
        out["B"] = np.min(adevs[i:j]) / BI_SCALE # estimation the floor allan deviation coefficent.
        fits["flat"] = (reg, np.min(adevs[i:j])) # stores the allan deviation of the floor or bias instabillity into the dict aka hashmap.

        tail = ok & (taus > taus[ok][-1] / 3.0) if ok.any() else ok # selects the largest tau valid points.
        if tail.any() and np.median(slopes[tail]) > 0.8: # high tau diagonistic check essentially seeing if the gradient is increasig steadily or median is above 0.8 and then warns.
            print(f"  [{label}] WARNING: long-tau slope ~ +1 ") 
        
    
    return out, (taus, adevs, errs, ok, fits) # return values.
# NOTE 3 more to do its just got too hot today to finish.

def plot_sensor(results_diag, title, ylabel, fname):
    fig, ax = plt.subplots(figsize = (10,6)) # creates an axis object and a figure object binding, start of the visualtion block.
    colours = plt.cm.tab10.colors # tab10 colourmap, a set of 10 distinct visually seperate colours, .colors pulls them out as a list of RBGA tuples.
    for k, (label, (taus, adevs, errs, ok, fits)) in enumerate(results_diag): # loops over multiple datasets/axes to plot them together. essentially pulling out 6 things used to analysis and diagnose that line, (label being the name given to the specfic plot line.)
        c = colours[k % 10] # picks this axis plot color from the 10-color palette.
        ax.loglog(taus[~ok], adevs[~ok], ".", color="0.8", ms=4) # quitely plots the points that failed the validaty check, shows where curves have invalid points, diagnostic tool
        ax.errorbar(taus[ok], adevs[ok], yerr=errs[ok], fmt=".", # plots the valid allan curve, plots the valid points from taus and adevs
                    color=c, ms=5, lw=0.8, label=label) #  ms = marker size 5, lw = error bar line width = 0.8
        if "white" in fits: # check to see whether white is present in the dict or aka the white noise section
            (i,j), N = fits["white"] # pulls the stored white-noise fit back out and unpacks it.
            tt = np.array([taus[i] / 2 , taus[j-1] * 2]) #  builds the x-coordinates for drawing the white-noise line.
            ax.loglog(tt, N / np.sqrt(tt), "--", color=c, lw=1.2) # the allan deviation which is the y axis which is calculate via: the white noise coefficent / square root of tau
        
        if "rrw" in fits: # check that random walk exist in the fits dict
            (i,j), v1 = fits["rrw"] # pulls the store random walk noise fit back out and unpacks it.
            tt = np.array([taus[i] / 2 , taus[j-1] * 2]) # builds the x coordinates for drawing the random walk line.
            ax.loglog(tt, N * np.sqrt(tt), "--", color=c, lw=1.2) # calcualtes the allan deviation via random walk coefficent / square root of tau and then plots the x axis (tau) and y axis (allan deviation)
# NOTE rrw is * and white is / because white is regressing from -0.5 to 0 whilst rrw is increasing from 0 -> 0.5 and beyond.
        if "flat" in fits: # checks the bias instabillity/floor exist in the fits dict
            (i,j), floor = fits["flat"] # pulls the stored bias isntabillity/floor back out and unpacks it.
            ax.hlines(floor, taus[i], taus[j - 1], color = c, lw=1.0, alpha = 0.6) # inputs directly because gradient is flat, eg the bias instabillity/floor should remain constant.
    
    ax.set_xlabel(r"averaging time $/tau$ (s)") # label for horizontal axel for this specfic axis, its tau.
    ax.set_ylabel(ylabel) # sets y axis label, this one more generic because it could be a range of different things eg accel(m/s), gyro (rad/s), etc
    ax.set_title(f"{title} (grey = excluded; -- white fit, : RRW fit)") # sets the plot title. title variable replaced by whatever is assigned to it
    ax.legend() # this displays the legend on the plot (maps the visual elements, is kinda like a key box. that maps names to colours.)
    ax.grid(True, which="both", alpha = 0.3) # adds gridlines to the plot, and then true turns the grid lines on alpha makes them 30 percent opaque.
    plt.tight_layout() # this adjusts the spacing so nothing gets clipped eg trimmed or overlaps with each other.
    fig.savefig(fname, dpi=150) # writes the figure to an image file, 150 dpi is 1500 x 900 pixels, (saves it as charts.)
    print(f"Saved {fname}")  # prints conformation message to terminal about the plots being saved.
    
def main(): # main thread definition
    imu = parse_log(LOG_PATH) # this reads and parses your imu log file into usable data.
    order, fs, _ = check_sampling(imu[:, 0]) # checks the timing of your samples and unpacks three things from the result. order : the chronology of the samples, fs : sample rate inferred from hz, _ discards the value.
    imu = imu[order] # rearranges all rows of imu into ascending time order, ascending by start -> finish essentially.

    if WARMUP_TRIM_S > 0: # essentially trimming the first few seconds of static interference some sensors can get on startup.
        keep = imu[:, 0] >= imu[0,0] + WARMUP_TRIM_S # builds a sub section of the total sample size that essentially disregards samples in the startup time.
        print(f"Trimmed")
        imu = imu[keep] # keeps only the post warmup samples.
    T = imu[-1, 0] - imu[0, 0] # computes total duration of samples from start to finish either trimmed or not.

    sensors = [
          ("gyro", ["gyro_x", "gyro_y", "gyro_z"], slice(1, 4), # essentially the slices selects from columns 1-3 and just grabs them essentially a multi-column read
         "rad/s", "Allan deviation (rad/s)"),
        ("accel", ["accel_x", "accel_y", "accel_z"], slice(4, 7), # same thing as the above one just for 4-6
         "m/s^2", "Allan deviation (m/s^2)"),
    ]
     
    all_results = {} # empty dict/hasmap
    for name, labels, cols, unit, ylabel in sensors: # unpacks 5 fields per sensor entry.
        diag = [] # creates a list/super vector.
        for i,label in enumerate(label): # tracks both the index and the value in label
            res,d = extract_axis(imu[:, cols][:, i], fs, T, label, unit) # runs the full extraction on one axis/one column and unpacks its two returns, the results dict, or the coefficents (white noise bias instabillity random walk) and diagonistic information for plotting and graph work.
            all_results[label] = res # storing the current axis results dict into the all results dict,
            diag.append((label, d)) # adds the current axis diagionist bundle to the diagionstic list or diag list.
        plot_sensor(diag, f"allan devation - {name}", ylabel,
                    f"{OUT_DIR}/allan_{name}_verified.png") # calls the plotting function for one whole sensor or 3 axis's on a shared figure and then saves it.
    
    print(f"{'axis':>8} | {'N (white)':>11} | {'K (RRW)':>11} | "
          f"{'B (BI)':>11} | {'sig/sample':>11} | {'sig bias/step':>13}")
    for label, r in all_results.items(): # loops over the result hashmap to print one table row per axis eg gyro_x
        print(f"{label:>8} | {r['N']:>11.3e} | {r['K']:>11.3e} | "
              f"{r['B']:>11.3e} | {r['sigma_white_per_sample']:>11.3e} | "
              f"{r['sigma_bias_step']:>13.3e}")
    print("""
Units: N in [unit]/sqrt(Hz) (= [unit]*sqrt(s)); K in [unit]/sqrt(s)
       (= [unit]*sqrt(Hz)); B in [unit].
Simulator / ESKF usage at sample rate fs:
  per-sample white measurement noise:  std = N * sqrt(fs)
  per-step bias random-walk increment: std = K * sqrt(dt) = K / sqrt(fs)
  continuous PSDs (if your Q is built in continuous time): N^2 and K^2.
NaN = that region was not observable in this record; do not invent a number.
""")
    
    with open(f"{OUT_DIR}/noise_params.json", "w") as f:
        json.dump({k: {kk: (None if isinstance(vv, float) and np.isnan(vv)else vv) for kk,vv in v.items()} for k,v in all_results.item()}, f , indent=2) # writes all the results to noise_params.json, converting any NaNs to JSON null along the way.


    plt.show() # displays the figures in a interactive window.









        







     




        







    

        

                

            






















    






        







