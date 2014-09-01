function [ad,feat] = test()
p = plexon_client();
p.add_chan([70,72,76]- 64 + 8,'replace_all');
p.send_marco()
p.get_polo(5)

% load('seba/decoders/simple_dec.mat')
% p_extractor = pow_extractor_pk(decoder.p_extractor_params.window_size,...
%                                       decoder.p_extractor_params.franges,...
%                                       decoder.baseline_used.power.frange,...
%                                       decoder.p_extractor_params.f_max,...
%                                       decoder.p_extractor_params.fs,...
%                                       decoder.p_extractor_params.used_chan);
% 
% used_chan = decoder.p_extractor_params.used_chan;                                  
% neural_buffer_t = 1;  % in secs
% neural_buffer_size = round(neural_buffer_t*1000);
% neural_buffer = ringbuffer(length(used_chan),neural_buffer_size);

if p.polo_recv ~=1
    disp('Didnt Connect')
    ad = 0;
    kin = 0;
else
    

    ad = [];
    kin = [];
    p.clear_buffer();
    for i = 1:100
        if mod(i, 10);
            p.keep_conn_alive()
        end
        t0 = GetSecs();
        t = tic;
        p.get_data(); 
        neur = p.get_neural();
        %size(neur,2)
        n_kin = p.get_kin();
        events = p.get_events();
        if sum(events)>0
            events
        end
        toc(t)

         ad = [ad neur];
        kin = [kin n_kin];
    end
    save('testdat.mat','ad','kin')
end

params     = struct('fpass',[0 120],'Fs',1000,'tapers',[3 5]);

tm = size(ad,2);
x=1:100:tm;
%feat = zeros(length(x)-1);

for xi = 1:length(x)-2
    ft = ad(:,x(xi):x(xi)+200);
    [S,f] = mtspectrumc(ft',params);
    inds = intersect(find(f>25),find(f<40));
    tmp = sum(S(inds));
    feat(xi)=tmp;
    
    inds = intersect(find(f>2),find(f<100));
    tmp = sum(S(inds));
    feat_tot(xi)=tmp;
end

end