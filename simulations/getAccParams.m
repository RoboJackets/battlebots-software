function acc_params = getAccParams(partname, userng)
%% GETACCPARAMS


g = 9.81; % acceleration due to gravity moron u surprised (m/s^2)/g

if strcmp(partname, "ADXL375")
    if userng
        mr = randrange(1, 1, 180, 200) .* g;
        reso = randrange(1, 1, 44, 54) .* 1e-3 .* g;
        constbias = randnorm(1, 3, -400, 400, -6e3, 6e3) .* 1e-3 .* g;
        axismis = abs(randnorm(1, 3, -2.5, 2.5, -inf, inf));
        noisedens = abs(randnorm(1, 3, -5, 5, -inf, inf)) .* 1e-3 .* g;
        tempbias = randnorm(1, 3, -10, 10, -inf, inf) .* 1e-3 .* g;
        tempsf = abs(randnorm(1, 3, -0.02, 0.02, -inf, inf));
    else
        mr = 200 .* g;
        reso = 49 .* 1e-3 .* g;
        constbias = [1 1 1] .* 400 .* 1e-3 .* g;
        axismis = [1 1 1] .* 2.5;
        noisedens = 5 .* 1e-3 .* g;
        tempbias = [1 1 1] .* 10 .* 1e-3 .* g;
        tempsf = 0.02 .* [1 1 1];
    end
    acc_params = accelparams( ...
        "MeasurementRange", mr, ...
        "Resolution", reso, ...
        "ConstantBias", constbias, ...
        "AxesMisalignment", axismis, ...
        "NoiseDensity", noisedens, ...
        "TemperatureBias", tempbias, ...
        "TemperatureScaleFactor", tempsf);
else
    % ideal accelerometer
    acc_params = accelparams();
end


end

function num = randrange(x, y, mini, maxi)

num = rand(x, y).*(maxi - mini) + mini;

end

function num = randnorm(x, y, mini, maxi, miniext, maxiext)

num = (maxi + mini) ./ 2 + (maxi - mini) ./ 2 .* randn(x, y);
num(num > maxiext) = maxiext;
num(num < miniext) = miniext;

end