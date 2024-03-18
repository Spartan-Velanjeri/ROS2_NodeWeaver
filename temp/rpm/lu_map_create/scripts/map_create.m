data = [];
resolv = 30;
noise = 3;

%%
x = 0:resolv:73510;
y = zeros(size(x));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data = [-1*x+x_noise;y+y_noise];

%%
y = resolv:resolv:2280;
x = zeros(size(y));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [x+x_noise+data(1,end);-1*y+y_noise+data(2,end)];
data = [data data_section];

%%
x = resolv:resolv:40000;
y = zeros(size(x));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
y = resolv:resolv:5370;
x = zeros(size(y));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));-1*y+y_noise+data(2,end)];
data = [data data_section];

%%
x = resolv:resolv:3500;
y = zeros(size(x));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
y = resolv:resolv:5370;
x = zeros(size(y));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
x = resolv:resolv:19900;
y = zeros(size(x));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
y = resolv:resolv:49130;
x = zeros(size(y));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));-1*y+y_noise+data(2,end)];
data = [data data_section];

%%
x = resolv:resolv:2300;
y = zeros(size(x));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
y = resolv:resolv:49130;
x = zeros(size(y));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
x = resolv:resolv:7690;
y = zeros(size(x));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [(x+x_noise+data(1,end));y+y_noise+data(2,end)];
data = [data data_section];

%%
y = resolv:resolv:2280;
x = zeros(size(y));
x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [x+x_noise+data(1,end);y+y_noise+data(2,end)];
data = [data data_section];

%%
x_c = 8050;
y_c = 250;
r = 350/2;
phi = 1:10:360;
x = r*cosd(phi) + x_c;
y = r*sind(phi) + y_c;

x_noise = noise*(2*(rand(size(x))-0.5));
y_noise = noise*(2*(rand(size(y))-0.5));
data_section = [-1*x+x_noise;-1*y+y_noise];

% subsample = 1;
% plot(data(1,1:subsample:end),data(2,1:subsample:end),'.k');
% axis equal;

data = [data data_section];

%%
x_c = 8050;
y_c = 250;

for i = 1:9
    x_c = x_c + 7200;
    y_c = 250;
    r = 350/2;
    phi = 1:10:360;
    x = r*cosd(phi) + x_c;
    y = r*sind(phi) + y_c;

    x_noise = noise*(2*(rand(size(x))-0.5));
    y_noise = noise*(2*(rand(size(y))-0.5));
    data_section = [-1*x+x_noise;-1*y+y_noise];

    % subsample = 1;
    % plot(data(1,1:subsample:end),data(2,1:subsample:end),'.k');
    % axis equal;

    data = [data data_section];
end

%%
x_c = 8050;
y_c = 250;

for i = 1:7
    y_c = y_c + 7200;
    
    r = 350/2;
    phi = 1:10:360;
    x = r*cosd(phi) + x_c;
    y = r*sind(phi) + y_c;

    x_noise = noise*(2*(rand(size(x))-0.5));
    y_noise = noise*(2*(rand(size(y))-0.5));    
    data_section = [-1*x+x_noise;-1*y+y_noise];

    % subsample = 1;
    % plot(data(1,1:subsample:end),data(2,1:subsample:end),'.k');
    % axis equal;

    data = [data data_section];
end

%% Plot

subsample = 1;
plot(data(1,1:subsample:end)/1e3,data(2,1:subsample:end)/1e3,'.k');
axis equal;
xlabel('x [m]');
ylabel('y [m]');
grid on;