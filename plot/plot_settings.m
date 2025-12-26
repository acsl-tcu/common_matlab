function settings = plot_settings(savefolder, savename, opt)
%plot_settings プロット関連のセッティングを行う関数
%   詳細説明をここに記述
arguments
    savefolder;
    savename;
    opt.phase = "f";
    opt.FontSize = 18;
    opt.LineWidth = 1.5;
end
settings.savefolder = savefolder;
settings.savename = savename;

settings.phase = opt.phase;
settings.FS = opt.FontSize;
settings.LW = opt.LineWidth;
end