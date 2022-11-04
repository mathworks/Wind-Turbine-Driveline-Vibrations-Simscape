# Wind Turbine Driveline with Vibrations in Simscape&trade;

[![View Wind Turbine Driveline with Vibrations on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/###-wind-turbine-driveline-vibrations-simscape)

Version 1.0 for MATLAB&reg; R2022b or newer

## Introduction

This example includes a wind turbine rotor and driveshaft
with optional transverse vibrations in Simscape Driveline&trade; and controllers in
Simulink&reg;. A live script describes the model components and demonstrates
workflows for analyzing the system performance.

The direct-drive wind turbine has a power capacity of 10 MW.
Controllers based on the NREL 5-MW reference wind turbine adjust the
generator torque and collective blade pitch.

The rotor and shaft are supported by 4 fluid film bearings.
This example lets you optionally model transverse vibrations of the
driveshaft due to a generator unbalanced magnetic force. The rotor dynamics 
can be modeled using a lumped mass finite element method or a reduced order 
model to simulate the shaft vibrations efficiently. The reduced order model 
can be a traditional static eigenmodes method, a speed-dependent eigenmodes 
method, or a Craig-Bampton method. 

## For MATLAB R2022b

Version 1.0 is available.
This version requires
[MATLAB](https://www.mathworks.com/products/matlab.html),
[Simulink](https://www.mathworks.com/products/simulink.html),
[Simscape](https://www.mathworks.com/products/simscape.html), and
[Simscape Driveline](https://www.mathworks.com/products/simscape-driveline.html).

To download the released version without Git repository data,
use the link below.

- https://github.com/mathworks/wind-turbine-driveline-vibrations-simscape/archive/refs/tags/v1.0.0.zip


## How to Use

Open the `WindTurbineDrivelineWithVibrationsMainScript.mlx` Live Script in 
MATLAB. The script contains a description of the model and
control buttons to run the model and scripts.

## How to Use in MATLAB Online

You can try this in [MATLAB Online][url_online].
In MATLAB Online, from the **HOME** tab in the toolstrip,
select **Add-Ons** &rarr; **Get Add-Ons**
to open the Add-On Explorer.
Then search for the submission name,
navigate to the submission page,
click **Add** button, and select **Save to MATLAB Drive**.

[url_online]:https://www.mathworks.com/products/matlab-online.html


## See Also

- [Wind Turbine Model][url-wtm]

[url-wtm]: https://www.mathworks.com/matlabcentral/fileexchange/25752-wind-turbine-model#readme

## License

See [`LICENSE.txt`](LICENSE.txt).

_Copyright 2022 The MathWorks, Inc._
