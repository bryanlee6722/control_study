function animate_cartpole(t, X, p)

    l = p.l;
    cart_w = 0.3;
    cart_h = 0.18;
    wheel_r = 0.05;

    figure;
    for k = 1:5:length(t)
        clf; hold on; axis equal;

        x = X(k,1);
        th = X(k,3);

        % cart
        rectangle('Position',[x-cart_w/2, -cart_h/2, cart_w, cart_h], ...
                  'Curvature', 0.05, 'LineWidth', 1.5);

        % wheels
        viscircles([x-cart_w/4, -cart_h/2-wheel_r], wheel_r, 'LineWidth', 1);
        viscircles([x+cart_w/4, -cart_h/2-wheel_r], wheel_r, 'LineWidth', 1);

        % pendulum pivot
        px = x;
        py = cart_h/2;

        % pendulum tip
        tipx = px + l*sin(th);
        tipy = py + l*cos(th);

        plot([px tipx], [py tipy], 'LineWidth', 2);
        plot(tipx, tipy, 'o', 'MarkerSize', 8, 'LineWidth', 1.5);

        % ground
        plot([-5 5], [-cart_h/2-wheel_r*2, -cart_h/2-wheel_r*2], 'k', 'LineWidth', 1);

        xlim([-2 2]);
        ylim([-0.4 1.0]);
        title(sprintf('t = %.2f s', t(k)));
        drawnow;
    end
end