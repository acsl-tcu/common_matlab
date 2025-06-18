function clear_axes(app)
cla(app.UIAxes); if ~isempty(app.UIAxes.Colorbar) app.UIAxes.Colorbar.Visible = 'off';end
cla(app.UIAxes2);
cla(app.UIAxes3);
cla(app.UIAxes4);
end