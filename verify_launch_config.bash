#!/bin/bash

# DJI EP Workspace Launch Configuration Verification Script
# This script verifies all launch files and configurations to prevent position overlap issues

echo "DJI EP Launch Configuration Verification"
echo "========================================"

# Define workspace paths
export USER_HOME="${HOME}"
export USER_WORKSPACE="${USER_HOME}/git/me"
export DJI_EP_WS="${USER_WORKSPACE}/Dji_EP_ws"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local status=$1
    local message=$2
    case $status in
        "OK")
            echo -e "${GREEN}✅ $message${NC}"
            ;;
        "WARN")
            echo -e "${YELLOW}⚠️  $message${NC}"
            ;;
        "ERROR")
            echo -e "${RED}❌ $message${NC}"
            ;;
        "INFO")
            echo -e "${BLUE}ℹ️  $message${NC}"
            ;;
    esac
}

# Function to check file existence
check_file() {
    local file_path=$1
    local description=$2
    
    if [ -f "$file_path" ]; then
        print_status "OK" "Found: $description"
        return 0
    else
        print_status "ERROR" "Missing: $description at $file_path"
        return 1
    fi
}

# Function to check executable permissions
check_executable() {
    local file_path=$1
    local description=$2
    
    if [ -x "$file_path" ]; then
        print_status "OK" "Executable: $description"
        return 0
    else
        print_status "WARN" "Not executable: $description"
        chmod +x "$file_path" 2>/dev/null && print_status "OK" "Fixed permissions for: $description"
        return 1
    fi
}

# Function to extract position parameters from launch file
extract_positions() {
    local launch_file=$1
    
    print_status "INFO" "Extracting positions from: $(basename $launch_file)"
    
    # Extract UAV positions
    local uav_x=$(grep -oP 'uav1_init_x.*default="\K[^"]*' "$launch_file" 2>/dev/null || echo "not found")
    local uav_y=$(grep -oP 'uav1_init_y.*default="\K[^"]*' "$launch_file" 2>/dev/null || echo "not found")
    local uav_z=$(grep -oP 'uav_init_z.*value="\K[^"]*' "$launch_file" 2>/dev/null || echo "not found")
    
    # Extract smartcar positions
    local car_x=$(grep -oP 'smartcar.*-x \K[0-9.-]+' "$launch_file" 2>/dev/null || echo "not found")
    local car_y=$(grep -oP 'smartcar.*-y \K[0-9.-]+' "$launch_file" 2>/dev/null || echo "not found")
    local car_z=$(grep -oP 'smartcar.*-z \K[0-9.-]+' "$launch_file" 2>/dev/null || echo "not found")
    
    echo "    UAV Position: x=$uav_x, y=$uav_y, z=$uav_z"
    echo "    Car Position: x=$car_x, y=$car_y, z=$car_z"
    
    # Check for potential overlaps
    if [ "$uav_x" != "not found" ] && [ "$car_x" != "not found" ] && [ "$uav_y" != "not found" ] && [ "$car_y" != "not found" ]; then
        local x_diff=$(echo "$uav_x - $car_x" | bc -l 2>/dev/null || echo "0")
        local y_diff=$(echo "$uav_y - $car_y" | bc -l 2>/dev/null || echo "0")
        
        # Simple overlap check (assuming bc is available)
        if command -v bc >/dev/null 2>&1; then
            local distance_2d=$(echo "sqrt(($x_diff)^2 + ($y_diff)^2)" | bc -l 2>/dev/null || echo "1.0")
            local distance_check=$(echo "$distance_2d >= 0.3" | bc -l 2>/dev/null || echo "1")
            
            if [ "$distance_check" = "1" ]; then
                print_status "OK" "Safe 2D distance: ${distance_2d}m"
            else
                print_status "WARN" "Close 2D distance: ${distance_2d}m (recommend ≥0.3m)"
            fi
        else
            print_status "WARN" "Cannot calculate distance (bc not available)"
        fi
    fi
    
    echo ""
}

# Function to check ROS dependencies
check_ros_deps() {
    print_status "INFO" "Checking ROS dependencies..."
    
    # Check if ROS is sourced
    if [ -z "$ROS_DISTRO" ]; then
        print_status "WARN" "ROS not sourced in current shell"
    else
        print_status "OK" "ROS $ROS_DISTRO detected"
    fi
    
    # Check for required packages
    local packages=("gazebo_ros" "mavros" "tf2_ros")
    for pkg in "${packages[@]}"; do
        if rospack find "$pkg" >/dev/null 2>&1; then
            print_status "OK" "Package found: $pkg"
        else
            print_status "ERROR" "Package missing: $pkg"
        fi
    done
    
    echo ""
}

# Function to verify workspace structure
verify_workspace() {
    print_status "INFO" "Verifying workspace structure..."
    
    # Check main directories
    local dirs=("src" "build" "devel")
    for dir in "${dirs[@]}"; do
        if [ -d "${DJI_EP_WS}/$dir" ]; then
            print_status "OK" "Directory exists: $dir"
        else
            print_status "WARN" "Directory missing: $dir"
        fi
    done
    
    # Check sim_pkg structure
    local sim_dirs=("launch" "scripts" "urdf" "worlds")
    for dir in "${sim_dirs[@]}"; do
        if [ -d "${DJI_EP_WS}/src/sim_pkg/$dir" ]; then
            print_status "OK" "sim_pkg/$dir exists"
        else
            print_status "ERROR" "sim_pkg/$dir missing"
        fi
    done
    
    echo ""
}

# Function to check script files
verify_scripts() {
    print_status "INFO" "Verifying script files..."
    
    # Main scripts
    local scripts=(
        "${DJI_EP_WS}/run_fixed_simulation.bash:Fixed simulation launcher"
        "${DJI_EP_WS}/check_simulation_fixed.py:Enhanced simulation checker"
        "${DJI_EP_WS}/src/sim_pkg/scripts/ground_truth_to_vision_fixed.py:Fixed ground truth converter"
        "${DJI_EP_WS}/src/sim_pkg/scripts/simulation_monitor.py:Simulation monitor"
        "${DJI_EP_WS}/src/sim_pkg/scripts/check_positions.py:Position checker"
    )
    
    for script_info in "${scripts[@]}"; do
        IFS=':' read -ra ADDR <<< "$script_info"
        local script_path="${ADDR[0]}"
        local script_desc="${ADDR[1]}"
        
        if check_file "$script_path" "$script_desc"; then
            check_executable "$script_path" "$script_desc"
        fi
    done
    
    echo ""
}

# Function to verify launch files
verify_launch_files() {
    print_status "INFO" "Verifying launch files..."
    
    # Launch files
    local launch_files=(
        "${DJI_EP_WS}/src/sim_pkg/launch/all_simulate_gazebo_fixed.launch:Fixed simulation launch"
        "${DJI_EP_WS}/src/sim_pkg/launch/all_simulate_gazebo.launch:Original simulation launch"
        "${DJI_EP_WS}/src/sim_pkg/launch/sitl_px4_indoor.launch:PX4 SITL launch"
    )
    
    for launch_info in "${launch_files[@]}"; do
        IFS=':' read -ra ADDR <<< "$launch_info"
        local launch_path="${ADDR[0]}"
        local launch_desc="${ADDR[1]}"
        
        if check_file "$launch_path" "$launch_desc"; then
            # Extract and verify positions
            extract_positions "$launch_path"
        fi
    done
    
    echo ""
}

# Function to check for potential conflicts
check_conflicts() {
    print_status "INFO" "Checking for potential conflicts..."
    
    # Check for running processes
    local running_procs=()
    
    if pgrep -f "gazebo" >/dev/null; then
        running_procs+=("Gazebo")
    fi
    
    if pgrep -f "px4" >/dev/null; then
        running_procs+=("PX4")
    fi
    
    if pgrep -f "roscore\|rosmaster" >/dev/null; then
        running_procs+=("ROS Core")
    fi
    
    if [ ${#running_procs[@]} -gt 0 ]; then
        print_status "WARN" "Running processes detected: ${running_procs[*]}"
        print_status "INFO" "Consider stopping them before launching new simulation"
    else
        print_status "OK" "No conflicting processes running"
    fi
    
    # Check for port conflicts
    local ports=(11311 4560 14540 14580)
    for port in "${ports[@]}"; do
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            print_status "WARN" "Port $port is in use"
        else
            print_status "OK" "Port $port available"
        fi
    done
    
    echo ""
}

# Function to validate configuration parameters
validate_config() {
    print_status "INFO" "Validating configuration parameters..."
    
    # Check fixed launch file for proper positioning
    local fixed_launch="${DJI_EP_WS}/src/sim_pkg/launch/all_simulate_gazebo_fixed.launch"
    
    if [ -f "$fixed_launch" ]; then
        # Check for proper launch-prefix delays
        local delays=$(grep -c "launch-prefix.*sleep" "$fixed_launch" 2>/dev/null || echo "0")
        if [ "$delays" -gt 3 ]; then
            print_status "OK" "Launch delays configured ($delays delays found)"
        else
            print_status "WARN" "Few launch delays found ($delays), may cause timing issues"
        fi
        
        # Check for ground truth converter configuration
        if grep -q "ground_truth_to_vision_fixed" "$fixed_launch"; then
            print_status "OK" "Using fixed ground truth converter"
        else
            print_status "WARN" "Using original ground truth converter"
        fi
        
        # Check for monitoring node
        if grep -q "simulation_monitor" "$fixed_launch"; then
            print_status "OK" "Simulation monitoring enabled"
        else
            print_status "WARN" "No simulation monitoring configured"
        fi
    fi
    
    echo ""
}

# Function to generate fix recommendations
generate_recommendations() {
    print_status "INFO" "Generating recommendations..."
    
    echo "🔧 RECOMMENDED ACTIONS:"
    echo "======================"
    
    # Check if fixed version should be used
    local fixed_launch="${DJI_EP_WS}/src/sim_pkg/launch/all_simulate_gazebo_fixed.launch"
    if [ -f "$fixed_launch" ]; then
        echo "1. Use the fixed simulation launcher:"
        echo "   ./run_fixed_simulation.bash"
        echo ""
    fi
    
    # Position verification
    echo "2. Verify positions before launch:"
    echo "   # After starting simulation, run:"
    echo "   rosrun sim_pkg check_positions.py"
    echo ""
    
    # Monitoring
    echo "3. Monitor simulation health:"
    echo "   python3 check_simulation_fixed.py"
    echo ""
    
    # Cleanup
    echo "4. If issues persist, clean restart:"
    echo "   pkill -f gazebo; pkill -f px4; pkill -f roscore"
    echo "   sleep 2"
    echo "   ./run_fixed_simulation.bash"
    echo ""
    
    # Manual positioning
    echo "5. Manual position adjustment (if needed):"
    echo "   # Edit launch file parameters:"
    echo "   # UAV: x=1.0, y=2.0, z=0.5"
    echo "   # Car: x=1.0, y=2.5, z=0.05"
    echo ""
}

# Main verification process
main() {
    echo ""
    print_status "INFO" "Starting comprehensive verification..."
    echo ""
    
    # Change to workspace directory
    if [ -d "$DJI_EP_WS" ]; then
        cd "$DJI_EP_WS"
        print_status "OK" "Working in: $DJI_EP_WS"
    else
        print_status "ERROR" "Workspace not found: $DJI_EP_WS"
        exit 1
    fi
    
    echo ""
    
    # Run all verification steps
    verify_workspace
    check_ros_deps
    verify_scripts
    verify_launch_files
    check_conflicts
    validate_config
    
    # Generate summary
    echo ""
    print_status "INFO" "Verification complete!"
    echo ""
    
    generate_recommendations
    
    # Final status
    echo ""
    echo "🎯 SUMMARY:"
    echo "==========="
    echo "Verification completed. Check above for any issues marked with ❌ or ⚠️"
    echo ""
    echo "To proceed with simulation:"
    echo "1. Address any ERROR (❌) items first"
    echo "2. Consider fixing WARN (⚠️) items for optimal performance"
    echo "3. Run: ./run_fixed_simulation.bash"
    echo ""
}

# Script entry point
if [ "${BASH_SOURCE[0]}" == "${0}" ]; then
    main "$@"
fi
