function settings = plot_settings(opt)
%plot_settings プロット関連のセッティングを行う関数
%   詳細説明をここに記述
arguments
    opt.phase = "f";
    opt.FontSize double = 18;
    opt.LineWidth double = 1.5;
    opt.TimeRange (1,2) double = [0,10];
    opt.LegendName cell
    opt.Target string = ["p", "v", "q", "w", "input", "input2:4", "p1-p2"];
    opt.att string = ["er", "er", "e", "e", "", "", "er"];
end

settings.phase = opt.phase;
settings.FS = opt.FontSize;
settings.LW = opt.LineWidth;
settings.LW_ref = opt.LineWidth*2/3;
settings.range = opt.TimeRange;
settings.lgd = opt.LegendName;
settings.target = opt.Target;
settings.att = opt.att;
end