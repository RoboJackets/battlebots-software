function mag_params = getMagParams(partname, adcbits, vrange, userng)
%% GETMAGPARAMS

gauss2uT = 100; % 1 Gauss = 100 uT
volt2bits = (2^adcbits - 1) / vrange; % dacbits over 5V range

if strcmp(partname, "HMC1052")
    if userng
        % lmao no, too lazy! %
    else
        mr = 6 .* gauss2uT;
        reso = mr / (2^(adcbits-1)); % negative too!
        constbias = [0 0 0];
        axismis = [0 0 0] .* 3;
        noisedens = (50 / 1e9 * (12/vrange) * gauss2uT) .* [1 1 1];
    end
    mag_params = magparams( ...
        "MeasurementRange", mr, ...
        "Resolution", reso, ...
        "ConstantBias", constbias, ...
        "AxesMisalignment", axismis, ...
        "NoiseDensity", noisedens);
else
    % ideal accelerometer
    mag_params = magparams();
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