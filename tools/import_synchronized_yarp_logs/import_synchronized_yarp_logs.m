function [ varargout ] = import_synchronized_yarp_logs( sample_period, initial_time_removed, final_time_removed, varargin )
% Import and synchronize a bunch of yarp logs produced by yarp dataDumper
% 
%   sample_period is the desired period of the synchronized data, expressed
%   in seconds
%   initial_time_removed and final_time_removed are the time period that should be removed from the dataset
%   then the rest of the arguments are a list of data.log files, see for example:
%
%   [ft_foot_left,ft_leg_left] = import_synchronized_yarp_logs(0.01,1.0,1.0,'dataDumperLeftLeg/ft_foot/data.log','dataDumperLeftLeg/ft_leg/data.log');
% 

% number of dataset to synchronize
n_datasets = size(varargin,2);

raw_data_log = cell(n_datasets,1);

for i = 1:n_datasets
    raw_data_log{i} = dlmread(varargin{i});
end

% assert that all timestamps are in order
for i = 1:n_datasets
    % varargin{i}
    assert(all(diff(raw_data_log{i}(:,2)) >= 0));
end

%drop initial and final samples for each dataset
for i = 1:n_datasets
    % estimate sampling period for each dataset 
    sampling_period = mean(diff(raw_data_log{i}(:,2)));
    initial_samples_dropped = ceil(initial_time_removed/sampling_period);
    final_samples_dropped = ceil(final_time_removed/sampling_period);
    raw_data_log{i} = raw_data_log{i}(initial_samples_dropped:end-final_samples_dropped,:);
end


% find the min and max timestamps
mins = nan(n_datasets,1);
maxs = nan(n_datasets,1);
for i = 1:n_datasets
    mins(i) = min(raw_data_log{i}(:,2));
    maxs(i) = max(raw_data_log{i}(:,2));
end
max_min = max(mins);
min_max = min(maxs);

syn_dataset_duration = min_max-max_min;
syn_dataset_samples = syn_dataset_duration/sample_period;

% Get the timestap of the synchronized dataset
syn_dataset_timestamps = max_min + sample_period*(0:(syn_dataset_samples-1));

% Resample the data log, by taking the sample closest to the desired timestamp 
% Alternative strategy could be used, for example interpolation. 
% We choose the easist option to avoid adding unnecessary complexity.

for i = 1:n_datasets
    data_channels = size(raw_data_log{i},2)-2;
    varargout{i} = zeros(size(syn_dataset_timestamps,2),data_channels+1);

    %check that the selected timestamp interval is actually contained
    %in the one of the considered dataset
    assert(syn_dataset_timestamps(1) >= raw_data_log{i}(1,2) );
    assert(syn_dataset_timestamps(end) <= raw_data_log{i}(end,2));
    
    new_data_index = 1;
    original_data_index = 1;
    for new_sample_ts = syn_dataset_timestamps
        %find the samples of the dataset that enclose the required
        %timestamp
        while( new_sample_ts > raw_data_log{i}(original_data_index+1,2) )
            original_data_index = original_data_index+1;
        end
        assert( new_sample_ts >= raw_data_log{i}(original_data_index,2));
        assert( new_sample_ts <= raw_data_log{i}(original_data_index+1,2));
        distance_previous = new_sample_ts - raw_data_log{i}(original_data_index,2);
        distance_successor =  raw_data_log{i}(original_data_index+1,2) - new_sample_ts;
        
        if( distance_previous < distance_successor )
            varargout{i}(new_data_index,:) = raw_data_log{i}(original_data_index,2:end);
        else
            varargout{i}(new_data_index,:) = raw_data_log{i}(original_data_index+1,2:end);
        end
        new_data_index = new_data_index+1;
    end
    
    %new_data_index
    %size(syn_dataset_timestamps,2)
    assert(new_data_index-1 == size(syn_dataset_timestamps,2));
end

end
