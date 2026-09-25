// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tidal_vehicle_interfaces:msg/TerrainState.idl
// generated code does not contain a copyright notice
#include "tidal_vehicle_interfaces/msg/detail/terrain_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `tide_state`
#include "rosidl_runtime_c/string_functions.h"

bool
tidal_vehicle_interfaces__msg__TerrainState__init(tidal_vehicle_interfaces__msg__TerrainState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tidal_vehicle_interfaces__msg__TerrainState__fini(msg);
    return false;
  }
  // tide_state
  if (!rosidl_runtime_c__String__init(&msg->tide_state)) {
    tidal_vehicle_interfaces__msg__TerrainState__fini(msg);
    return false;
  }
  // tide_risk
  // water_level_m
  // tide_rate_m_per_minute
  // seconds_until_corridor_unsafe
  // corridor_traversable
  return true;
}

void
tidal_vehicle_interfaces__msg__TerrainState__fini(tidal_vehicle_interfaces__msg__TerrainState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // tide_state
  rosidl_runtime_c__String__fini(&msg->tide_state);
  // tide_risk
  // water_level_m
  // tide_rate_m_per_minute
  // seconds_until_corridor_unsafe
  // corridor_traversable
}

bool
tidal_vehicle_interfaces__msg__TerrainState__are_equal(const tidal_vehicle_interfaces__msg__TerrainState * lhs, const tidal_vehicle_interfaces__msg__TerrainState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // tide_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tide_state), &(rhs->tide_state)))
  {
    return false;
  }
  // tide_risk
  if (lhs->tide_risk != rhs->tide_risk) {
    return false;
  }
  // water_level_m
  if (lhs->water_level_m != rhs->water_level_m) {
    return false;
  }
  // tide_rate_m_per_minute
  if (lhs->tide_rate_m_per_minute != rhs->tide_rate_m_per_minute) {
    return false;
  }
  // seconds_until_corridor_unsafe
  if (lhs->seconds_until_corridor_unsafe != rhs->seconds_until_corridor_unsafe) {
    return false;
  }
  // corridor_traversable
  if (lhs->corridor_traversable != rhs->corridor_traversable) {
    return false;
  }
  return true;
}

bool
tidal_vehicle_interfaces__msg__TerrainState__copy(
  const tidal_vehicle_interfaces__msg__TerrainState * input,
  tidal_vehicle_interfaces__msg__TerrainState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // tide_state
  if (!rosidl_runtime_c__String__copy(
      &(input->tide_state), &(output->tide_state)))
  {
    return false;
  }
  // tide_risk
  output->tide_risk = input->tide_risk;
  // water_level_m
  output->water_level_m = input->water_level_m;
  // tide_rate_m_per_minute
  output->tide_rate_m_per_minute = input->tide_rate_m_per_minute;
  // seconds_until_corridor_unsafe
  output->seconds_until_corridor_unsafe = input->seconds_until_corridor_unsafe;
  // corridor_traversable
  output->corridor_traversable = input->corridor_traversable;
  return true;
}

tidal_vehicle_interfaces__msg__TerrainState *
tidal_vehicle_interfaces__msg__TerrainState__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tidal_vehicle_interfaces__msg__TerrainState * msg = (tidal_vehicle_interfaces__msg__TerrainState *)allocator.allocate(sizeof(tidal_vehicle_interfaces__msg__TerrainState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tidal_vehicle_interfaces__msg__TerrainState));
  bool success = tidal_vehicle_interfaces__msg__TerrainState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tidal_vehicle_interfaces__msg__TerrainState__destroy(tidal_vehicle_interfaces__msg__TerrainState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tidal_vehicle_interfaces__msg__TerrainState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tidal_vehicle_interfaces__msg__TerrainState__Sequence__init(tidal_vehicle_interfaces__msg__TerrainState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tidal_vehicle_interfaces__msg__TerrainState * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(tidal_vehicle_interfaces__msg__TerrainState)) {
      return false;
    }
    data = (tidal_vehicle_interfaces__msg__TerrainState *)allocator.zero_allocate(size, sizeof(tidal_vehicle_interfaces__msg__TerrainState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tidal_vehicle_interfaces__msg__TerrainState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tidal_vehicle_interfaces__msg__TerrainState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
tidal_vehicle_interfaces__msg__TerrainState__Sequence__fini(tidal_vehicle_interfaces__msg__TerrainState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      tidal_vehicle_interfaces__msg__TerrainState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

tidal_vehicle_interfaces__msg__TerrainState__Sequence *
tidal_vehicle_interfaces__msg__TerrainState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tidal_vehicle_interfaces__msg__TerrainState__Sequence * array = (tidal_vehicle_interfaces__msg__TerrainState__Sequence *)allocator.allocate(sizeof(tidal_vehicle_interfaces__msg__TerrainState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tidal_vehicle_interfaces__msg__TerrainState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tidal_vehicle_interfaces__msg__TerrainState__Sequence__destroy(tidal_vehicle_interfaces__msg__TerrainState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tidal_vehicle_interfaces__msg__TerrainState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tidal_vehicle_interfaces__msg__TerrainState__Sequence__are_equal(const tidal_vehicle_interfaces__msg__TerrainState__Sequence * lhs, const tidal_vehicle_interfaces__msg__TerrainState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tidal_vehicle_interfaces__msg__TerrainState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tidal_vehicle_interfaces__msg__TerrainState__Sequence__copy(
  const tidal_vehicle_interfaces__msg__TerrainState__Sequence * input,
  tidal_vehicle_interfaces__msg__TerrainState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(tidal_vehicle_interfaces__msg__TerrainState)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(tidal_vehicle_interfaces__msg__TerrainState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tidal_vehicle_interfaces__msg__TerrainState * data =
      (tidal_vehicle_interfaces__msg__TerrainState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tidal_vehicle_interfaces__msg__TerrainState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tidal_vehicle_interfaces__msg__TerrainState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tidal_vehicle_interfaces__msg__TerrainState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
