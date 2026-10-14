function availability = check_rainflow_availability()
%CHECK_RAINFLOW_AVAILABILITY Verify official MATLAB rainflow availability.
% No custom or fallback rainflow implementation is permitted by this project.

FunctionAvailable = exist('rainflow', 'file') == 2;
FunctionPath = string(which('rainflow'));
LicenseAvailable = logical(license('test', 'Signal_Toolbox'));
if LicenseAvailable
    LicenseCheckoutSucceeded = logical(license('checkout', 'Signal_Toolbox'));
else
    LicenseCheckoutSucceeded = false;
end
installed = ver('signal');
ProductInstalled = ~isempty(installed);
if ProductInstalled
    ProductName = string(installed(1).Name);
    ProductVersion = string(installed(1).Version);
else
    ProductName = "Signal Processing Toolbox";
    ProductVersion = "";
end
ReadyForRainflowAnalysis = FunctionAvailable && LicenseAvailable && ...
    LicenseCheckoutSucceeded && ProductInstalled;

if ReadyForRainflowAnalysis
    Status = "available";
    Limitation = "";
else
    Status = "blocked";
    Limitation = "MATLAB rainflow is unavailable because Signal Processing " + ...
        "Toolbox is not installed or not on the MATLAB path. The license " + ...
        "feature is available, but no custom rainflow substitute was used.";
end

availability = table(Status, ReadyForRainflowAnalysis, FunctionAvailable, ...
    LicenseAvailable, LicenseCheckoutSucceeded, ProductInstalled, ...
    ProductName, ProductVersion, ...
    FunctionPath, Limitation);
end
