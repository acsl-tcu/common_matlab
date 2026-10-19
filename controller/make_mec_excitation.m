function profile = make_mec_excitation(seed,setting)
% 軌道とは独立した乱数列で、4入力それぞれの周波数・位相を決める。
validateattributes(seed,{'numeric'},{'scalar','integer','nonnegative','<=',2^32-1});
validateattributes(setting.amplitude,{'numeric'},{'real','finite','size',[4,1],'nonnegative'});
validateattributes(setting.frequency_range,{'numeric'},{'real','finite','size',[1,2],'positive'});
assert(setting.frequency_range(2)>setting.frequency_range(1), ...
    'make_mec_excitation:InvalidFrequency','周波数の上限は下限より大きくしてください。');
validateattributes(setting.component_count,{'numeric'},{'scalar','integer','positive'});
validateattributes(setting.ramp_time,{'numeric'},{'scalar','finite','positive'});
validateattributes(setting.flight_time,{'numeric'},{'scalar','finite','positive'});
assert(2*setting.ramp_time<=setting.flight_time, ...
    'make_mec_excitation:InvalidRamp','立ち上げ時間は飛行時間の半分以下にしてください。');
profile.seed = mod(double(seed)+104729,2^32);
stream = RandStream('mt19937ar','Seed',profile.seed);
profile.amplitude = setting.amplitude;
profile.frequency = setting.frequency_range(1)+diff(setting.frequency_range)* ...
    rand(stream,4,setting.component_count);
profile.phase = 2*pi*rand(stream,4,setting.component_count);
profile.ramp_time = setting.ramp_time;
profile.duration = setting.flight_time;
end
