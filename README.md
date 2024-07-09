# HR_Matlab
Code and results from a robust HR algorithm


STEP 1  Open video

STEP 2 Detect face and select corner points. Corner points include many non-valid points, e.g. Glasses, hair, beard etc. Only skin points are needed, an extra filter calibrated to skin colors was implemented in this example to select only skin points. Possible an initial calibration for user skin is needed


![image](https://github.com/mauOrtRuiz/HR_Matlab/assets/44585823/a964c146-68bb-43ff-b061-6f931e2c0555)

STEP 3  Green Channel is extracted, according to Rubinstein, green light has better result for HR determination, raw green intensity is extracted

![image](https://github.com/mauOrtRuiz/HR_Matlab/assets/44585823/af016dbb-f8a5-47f1-a2be-5649b2d32e88)

STEP 4 A detrend filter removes the trend in the signal

![image](https://github.com/mauOrtRuiz/HR_Matlab/assets/44585823/c909be29-8801-4804-a453-6035ab6f81e8)

STEP 5 Average filter is applied to smooth the signal

![image](https://github.com/mauOrtRuiz/HR_Matlab/assets/44585823/06bca619-d58e-4187-8ecf-3e6584a11cc7)

STEP 6 Determine Power Spectra by Welch's Method


