# Running SVD Timing Tests

To change frequency, edit "localize_freq" variable in visp_snail.cpp on line 110. 
https://github.com/secret-snail/snail/blob/435759ba3f65d691ccc8afe7638fc9fe47217ef8/src/visp_snail.cpp#LL110C6-L110C6

Timing CSV File starts in visp_snail.cpp on line 263.
https://github.com/secret-snail/snail/blob/435759ba3f65d691ccc8afe7638fc9fe47217ef8/src/visp_snail.cpp#LL263C25-L263C25

Total localization time is captured in visp_snail.cpp.
Start: Line 343 https://github.com/secret-snail/snail/blob/435759ba3f65d691ccc8afe7638fc9fe47217ef8/src/visp_snail.cpp#LL343C7-L343C7
Stop/Record: Line 348 https://github.com/secret-snail/snail/blob/435759ba3f65d691ccc8afe7638fc9fe47217ef8/src/visp_snail.cpp#L348


SVD time is captured in invert.hpp
Start: Line 146 https://github.com/secret-snail/snail/blob/435759ba3f65d691ccc8afe7638fc9fe47217ef8/src/cleartext-ref/invert.hpp#L146
Stop/Record: Line 148 https://github.com/secret-snail/snail/blob/435759ba3f65d691ccc8afe7638fc9fe47217ef8/src/cleartext-ref/invert.hpp#L148

To zero out pose, set variable "x[]" on line 338 of visp_snail.cpp as such: float _x[] = {0, 0, 0, 0, 0, 1}
To have persistent pose, set variable "x[]" on line 338 of visp_snail.cpp as such: float _x[] = {s_rvec[0], s_rvec[1], s_rvec[2], s_tvec[0], s_tvec[1], s_tvec[2]}
