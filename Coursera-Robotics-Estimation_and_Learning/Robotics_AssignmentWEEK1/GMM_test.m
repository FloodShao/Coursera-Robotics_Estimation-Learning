%step 1: choose initial values for the parameters

%set 'num_samples' to the number of sample points
num_samples = size(Samples, 1);
%the number of clusters
clusters = 2; 
%the vector lengths
n = 3;
%set tolerence
error = 1.0e-6;

%randomly select clusters data points to serve as the initial means
index = randperm(num_samples);
mu = Samples(index(1:clusters),:); %to select clusters data points as means

sigma = [];

%use overall covariance of the dataset as the initial variance for each
%cluster
for j = 1:clusters
    sigma{j} = cov(Samples);
end

%assign equal prior probabilities to each cluster
phi = ones(1, clusters) * (1/clusters);

%step 2: run EM algorithnm
for iter = 1:1000
    fprintf('EM Iteration %d\n', iter);
    
    %%Expectation:
    %calculate the probability for each data point for each distribution
    %matrix to hold the pdf value for each every data point for every
    %cluster
    pdf = zeros(num_samples, clusters);
    
    %For each cluster
    for i = 1:clusters
        %evaluate the gaussian for all data points for cluster 'i'
        pdf(:,i) = gaussianND(Samples, mu(i,:), sigma{i});
    end
    
    %multiply each pdf value by the prior probability for cluster
    pdf_w = bsxfun(@times, pdf, phi);
    %pdf [m x k]
    %phi [1 x k]
    %pdf_w [m x k]
    
    %divide the weighted probabilities by the sum of weighted probabilities
    %for each cluster
    W = bsxfun(@rdivide, pdf_w, sum(pdf_w, 2));

    %%Maximum
    %calculate the probability for each data point for each distribution
    
    %store the previous means
    PreMu = mu;
    
    %for each of the clusters
    for i = 1:clusters
        
        %calculate the prior probability for cluster 'i'
        phi(i) = mean(W(:,i), 1);
        
        %calculate the new mean for cluster 'i' by taking the weighted
        %average of all data points
        mu(i,:) = weightedAverage(W(:,i), Samples);

        %calculate the covariance matrix for cluster 'i' by taking the
        %weighted average of the covariance for each training example
        
        sigma_k = zeros(n,n);
        
        %subtract the cluster mean from all data points
        xm = bsxfun(@minus, Samples, mu(i,:));
        
        %calculate the contribution of each training example to the
        %covatiance matrix
        
        for  j = 1:num_samples
            sigma_k = sigma_k + (W(j,i) .* (xm(j,:)' * xm(j,:)));
        end
        
        %divide by the sum of weights
        sigma{i} = sigma_k ./ sum(W(:,i));
    end
    
    %check for convergence

    if (mu-PreMu) < error
        break;
    end
end

    