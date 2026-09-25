// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tidal_vehicle_interfaces:msg/VehicleHealth.idl
// generated code does not contain a copyright notice
#include "tidal_vehicle_interfaces/msg/detail/vehicle_health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `fault`
#include "rosidl_runtime_c/string_functions.h"

bool
tidal_vehicle_interfaces__msg__VehicleHealth__init(tidal_vehicle_interfaces__msg__VehicleHealth * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tidal_vehicle_interfaces__msg__VehicleHealth__fini(msg);
    return false;
  }
  // battery_percent
  // return_reserve_percent
  // mobility_health_percent
  // link_ok
  // payload_secured
  // fault
  if (!rosidl_runtime_c__String__init(&msg->fault)) {
    tidal_vehicle_interfaces__msg__VehicleHealth__fini(msg);
    return false;
  }
  return true;
}

void
tidal_vehicle_interfaces__msg__VehicleHealth__fini(tidal_vehicle_interfaces__msg__VehicleHealth * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // battery_percent
  // return_reserve_percent
  // mobility_health_percent
  // link_ok
  // payload_secured
  // fault
  rosidl_runtime_c__String__fini(&msg->fault);
}

bool
tidal_vehicle_interfaces__msg__VehicleHealth__are_equal(const tidal_vehicle_interfaces__msg__VehicleHealth * lhs, const tidal_vehicle_interfaces__msg__VehicleHealth * rhs)
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
  // battery_percent
  if (lhs->battery_percent != rhs->battery_percent) {
    return false;
  }
  // return_reserve_percent
  if (lhs->return_reserve_percent != rhs->return_reserve_percent) {
    return false;
  }
  // mobility_health_percent
  if (lhs->mobility_health_percent != rhs->mobility_health_percent) {
    return false;
  }
  // link_ok
  if (lhs->link_ok != rhs->link_ok) {
    return false;
  }
  // payload_secured
  if (lhs->payload_secured != rhs->payload_secured) {
    return false;
  }
  // fault
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->fault), &(rhs->fault)))
  {
    return false;
  }
  return true;
}

bool
tidal_vehicle_interfaces__msg__VehicleHealth__copy(
  const tidal_vehicle_interfaces__msg__VehicleHealth * input,
  tidal_vehicle_interfaces__msg__VehicleHealth * output)
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
  // battery_percent
  output->battery_percent = input->battery_percent;
  // return_reserve_percent
  output->return_reserve_percent = input->return_reserve_percent;
  // mobility_health_percent
  output->mobility_health_percent = input->mobility_health_percent;
  // link_ok
  output->link_ok = input->link_ok;
  // payload_secured
  output->payload_secured = input->payload_secured;
  // fault
  if (!rosidl_runtime_c__String__copy(
      &(input->fault), &(output->fault)))
  {
    return false;
  }
  return true;
}

tidal_vehicle_interfaces__msg__VehicleHealth *
tidal_vehicle_interfaces__msg__VehicleHealth__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tidal_vehicle_interfaces__msg__VehicleHealth * msg = (tidal_vehicle_interfaces__msg__VehicleHealth *)allocator.allocate(sizeof(tidal_vehicle_interfaces__msg__VehicleHealth), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tidal_vehicle_interfaces__msg__VehicleHealth));
  bool success = tidal_vehicle_interfaces__msg__VehicleHealth__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tidal_vehicle_interfaces__msg__VehicleHealth__destroy(tidal_vehicle_interfaces__msg__VehicleHealth * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tidal_vehicle_interfaces__msg__VehicleHealth__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__init(tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tidal_vehicle_interfaces__msg__VehicleHealth * data = NULL;

  if (size) {
    if (size > SIZE_MAX / sizeof(tidal_vehicle_interfaces__msg__VehicleHealth)) {
      return false;
    }
    data = (tidal_vehicle_interfaces__msg__VehicleHealth *)allocator.zero_allocate(size, sizeof(tidal_vehicle_interfaces__msg__VehicleHealth), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tidal_vehicle_interfaces__msg__VehicleHealth__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tidal_vehicle_interfaces__msg__VehicleHealth__fini(&data[i - 1]);
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
tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__fini(tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * array)
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
      tidal_vehicle_interfaces__msg__VehicleHealth__fini(&array->data[i]);
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

tidal_vehicle_interfaces__msg__VehicleHealth__Sequence *
tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * array = (tidal_vehicle_interfaces__msg__VehicleHealth__Sequence *)allocator.allocate(sizeof(tidal_vehicle_interfaces__msg__VehicleHealth__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__destroy(tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__are_equal(const tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * lhs, const tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tidal_vehicle_interfaces__msg__VehicleHealth__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tidal_vehicle_interfaces__msg__VehicleHealth__Sequence__copy(
  const tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * input,
  tidal_vehicle_interfaces__msg__VehicleHealth__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    if (input->size > SIZE_MAX / sizeof(tidal_vehicle_interfaces__msg__VehicleHealth)) {
      return false;
    }
    const size_t allocation_size =
      input->size * sizeof(tidal_vehicle_interfaces__msg__VehicleHealth);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tidal_vehicle_interfaces__msg__VehicleHealth * data =
      (tidal_vehicle_interfaces__msg__VehicleHealth *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tidal_vehicle_interfaces__msg__VehicleHealth__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tidal_vehicle_interfaces__msg__VehicleHealth__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tidal_vehicle_interfaces__msg__VehicleHealth__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
