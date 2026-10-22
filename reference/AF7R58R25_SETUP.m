function Q = AF7R58R25_SETUP()
% Run ONCE after overwriting files from the ZIP. No C++ changes / no mex.
Q=AF7R58R25_INSTALL_LOGGER();
Q.preflight=AF7R58R25_PREFLIGHT();
end
