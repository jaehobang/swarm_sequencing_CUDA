INCS =
#DEFS = -NDEBUG
CXXFLAGS = $(INCS) $(DEFS) -O2 -std=c++11 
LIBS = -ltbb -lrt -lcudart -lcudadevrt
LDFLAGS = -L/usr/local/cuda-8.0/targets/x86_64-linux/lib $(LIBS)
CUDA_SRCS = main.cu planner_interface.cu planner.cu simulator.cu flock_biased.cu formation_control.cu rendezvous.cu planner_pipeline.cu simulator_pipeline.cu
OBJS = 

NVCC = nvcc
CUDA_FLAGS = -std=c++11 
CUDA_FLAGS += --relocatable-device-code=true
#CUDA_FLAGS += --expt-relaxed-constexpr
#CUDA_FLAGS += -gencode arch=compute_10,code=sm_10 
#CUDA_FLAGS += -gencode arch=compute_20,code=sm_20 
#CUDA_FLAGS += -gencode arch=compute_30,code=sm_30 
#CUDA_FLAGS += -gencode arch=compute_35,code=[compute_35,sm_35]
CUDA_FLAGS += -gencode arch=compute_50,code=[compute_50,sm_50,sm_52] 
#CUDA_FLAGS += -gencode arch=compute_53,code=sm_53
CUDA_FLAGS += -I /home/jaeho-linux/hri2017/src/backend_S/src/cub-1.7.4

PROG = swarm_planner

all: $(PROG)


$(PROG): $(CUDA_SRCS)
	$(NVCC) $(CUDA_FLAGS) -o $@ $^

clean:
	rm -f $(PROG) *.o 
