function clear_axes(app)
cla(app.UIAxes); if ~isempty(app.UIAxes.Colorbar) app.UIAxes.Colorbar.Visible = 'off';end
end